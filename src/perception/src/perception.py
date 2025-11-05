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
        self.resolution = 0.1  # size of each voxel (meters)
        self.max_range = 5.0   # meters around the drone to consider

        # --- Subscriptions ---
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        # --- Publishers ---
        self.map_pub = self.create_publisher(OccupancyGrid, '/perception/occupancy', 10)

        self.get_logger().info("🧠 Perception node running. Subscribed to /camera/depth/color/points")

    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to numpy array (x, y, z)
        points = np.array([p[:3] for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)])
        if len(points) == 0:
            return

        # Limit range (ignore faraway points)
        dists = np.linalg.norm(points, axis=1)
        points = points[dists < self.max_range]

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
        msg_out.header.frame_id = "map"
        msg_out.info.resolution = self.resolution
        msg_out.info.width = grid_size
        msg_out.info.height = grid_size
        msg_out.info.origin.position.x = -self.max_range
        msg_out.info.origin.position.y = -self.max_range
        msg_out.info.origin.position.z = 0.0

        msg_out.data = grid.flatten().tolist()

        self.map_pub.publish(msg_out)

        self.get_logger().info_throttle(5.0, f"Published occupancy grid ({grid_size}x{grid_size})")

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
