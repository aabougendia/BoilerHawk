#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # --- Parameters ---
        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        self.declare_parameter('occupancy_topic', '/perception/occupancy')
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('grid_frame', 'map')

        self.pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)
        self.occupancy_topic = str(self.get_parameter('occupancy_topic').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.grid_frame = str(self.get_parameter('grid_frame').value)

        if self.resolution <= 0.0:
            self.get_logger().warning('Resolution must be positive; defaulting to 0.1 m')
            self.resolution = 0.1
        if self.max_range <= 0.0:
            self.get_logger().warning('Max range must be positive; defaulting to 5.0 m')
            self.max_range = 5.0

        # --- Subscriptions ---
        self.subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self.pointcloud_callback,
            10
        )

        # --- Publishers ---
        self.map_pub = self.create_publisher(OccupancyGrid, self.occupancy_topic, 10)

        self._last_grid_log_time = None

        self.get_logger().info(
            f"🧠 Perception node running. Listening on {self.pointcloud_topic} and publishing {self.occupancy_topic}"
        )

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

        # Shift coordinates to make the map centered around drone
        # (assuming drone is at origin)
        origin = np.array([0.0, 0.0, 0.0])
        points = points - origin

        # Create 2D occupancy projection (top-down)
        # Drop Z coordinate and map X,Y to grid cells
        grid_size = int((self.max_range * 2) / self.resolution)
        grid = np.zeros((grid_size, grid_size), dtype=np.int8)

        # Convert points to voxel indices
        half = grid_size // 2
        x_idx = (points[:, 0] / self.resolution + half).astype(int)
        y_idx = (points[:, 1] / self.resolution + half).astype(int)

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
        msg_out.info.origin.position.x = -self.max_range
        msg_out.info.origin.position.y = -self.max_range
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
