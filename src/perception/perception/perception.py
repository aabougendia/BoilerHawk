#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # --- Parameters ---
        self.declare_parameter('pointcloud_topic', '/camera/points')
        self.declare_parameter('occupancy_topic', '/occupancy_grid')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('grid_frame', 'map')

        self.pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)
        self.occupancy_topic = str(self.get_parameter('occupancy_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.grid_frame = str(self.get_parameter('grid_frame').value)

        if self.resolution <= 0.0:
            self.get_logger().warning('Resolution must be positive; defaulting to 0.1 m')
            self.resolution = 0.1
        if self.max_range <= 0.0:
            self.get_logger().warning('Max range must be positive; defaulting to 5.0 m')
            self.max_range = 5.0

        # --- Drone pose state ---
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_yaw = 0.0  # radians

        # --- Subscriptions ---
        self.subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )

        # MAVROS local position (BEST_EFFORT to match MAVROS QoS)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_cb,
            mavros_qos,
        )

        # --- Publishers ---
        self.map_pub = self.create_publisher(OccupancyGrid, self.occupancy_topic, 10)

        self._last_grid_log_time = None

        self.get_logger().info(
            f"Perception node running. Listening on {self.pointcloud_topic} "
            f"and {self.pose_topic}, publishing {self.occupancy_topic}"
        )

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        """Update drone position from MAVROS local position (ENU)."""
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array (x, y, z)
        points = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.asarray(points, dtype=np.float32)
        if len(points) == 0:
            return

        # Limit range (ignore faraway points)
        dists = np.linalg.norm(points, axis=1)
        points = points[dists < self.max_range]
        if len(points) == 0:
            return

        # ---------------------------------------------------------
        # Transform camera-frame points into MAVROS local ENU frame
        # Camera frame: x-right, y-down, z-forward (optical)
        # but Gazebo rgbd_camera already outputs in sensor frame
        # which after the joint is approximately: x-forward, y-left, z-up
        # We rotate by drone yaw, then translate by drone position.
        # ---------------------------------------------------------
        cos_yaw = np.cos(self.drone_yaw)
        sin_yaw = np.sin(self.drone_yaw)

        # Rotate points by drone yaw (2D rotation around Z)
        rx = cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
        ry = sin_yaw * points[:, 0] + cos_yaw * points[:, 1]

        # Translate to drone position in MAVROS local frame
        world_x = rx + self.drone_x
        world_y = ry + self.drone_y

        # ---------------------------------------------------------
        # Build occupancy grid centered on the drone in ENU coords
        # Grid spans [drone_x - max_range, drone_x + max_range] etc.
        # ---------------------------------------------------------
        grid_size = int((self.max_range * 2) / self.resolution)
        grid = np.zeros((grid_size, grid_size), dtype=np.int8)

        # Grid origin in world ENU coordinates
        origin_x = self.drone_x - self.max_range
        origin_y = self.drone_y - self.max_range

        # Convert world points to grid indices
        x_idx = ((world_x - origin_x) / self.resolution).astype(int)
        y_idx = ((world_y - origin_y) / self.resolution).astype(int)

        # Clip indices to stay inside map
        valid = (x_idx >= 0) & (x_idx < grid_size) & (y_idx >= 0) & (y_idx < grid_size)
        x_idx, y_idx = x_idx[valid], y_idx[valid]

        # Mark occupied cells (100 means occupied)
        grid[y_idx, x_idx] = 100

        # Convert to OccupancyGrid message
        msg_out = OccupancyGrid()
        msg_out.header = Header()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = self.grid_frame
        msg_out.info.resolution = self.resolution
        msg_out.info.width = grid_size
        msg_out.info.height = grid_size
        msg_out.info.origin.position.x = origin_x
        msg_out.info.origin.position.y = origin_y
        msg_out.info.origin.position.z = 0.0
        msg_out.info.origin.orientation.w = 1.0

        msg_out.data = grid.flatten().tolist()

        self.map_pub.publish(msg_out)

        now = self.get_clock().now()
        if (
            self._last_grid_log_time is None
            or (now - self._last_grid_log_time).nanoseconds >= 5_000_000_000
        ):
            self.get_logger().info(f"Published occupancy grid ({grid_size}x{grid_size})")
            self._last_grid_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
