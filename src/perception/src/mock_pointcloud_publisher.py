#!/usr/bin/env python3
"""Synthetic PointCloud2 publisher that mimics a RealSense D435 depth stream."""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2


class MockPointCloudPublisher(Node):
    """Publishes a dynamic mock point cloud for testing perception without hardware."""

    def __init__(self) -> None:
        super().__init__('mock_pointcloud_publisher')

        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('points_per_frame', 6000)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('drone_height', 1.2)
        self.declare_parameter('cone_tip_distance', 0.5)
        self.declare_parameter('cone_length', 2.0)
        self.declare_parameter('cone_base_radius', 1.0)

        self.publish_frequency = float(self.get_parameter('publish_frequency').value)
        self.points_per_frame = int(self.get_parameter('points_per_frame').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.drone_height = float(self.get_parameter('drone_height').value)
        self.cone_tip_distance = float(self.get_parameter('cone_tip_distance').value)
        self.cone_length = float(self.get_parameter('cone_length').value)
        self.cone_base_radius = float(self.get_parameter('cone_base_radius').value)

        # Fallbacks if parameters were not set (e.g. integer->0)
        if self.points_per_frame <= 0:
            self.points_per_frame = 6000
        if self.publish_frequency <= 0:
            self.publish_frequency = 10.0

        self.publisher = self.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
        self.timer = self.create_timer(1.0 / self.publish_frequency, self._publish_mock_cloud)

        self.rng = np.random.default_rng()
        self.get_logger().info(
            '📡 Mock point cloud publisher online. Publishing to /camera/depth/color/points'
        )

    def _publish_mock_cloud(self) -> None:
        points = self._build_scene()
        if points.size == 0:
            return

        # Add mild Perlin-like jitter via colored noise to emulate sensor artifacts.
        noise = self.rng.normal(0.0, 0.01, size=points.shape)
        noisy_points = points + noise.astype(np.float32)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        cloud_msg = pc2.create_cloud_xyz32(header, noisy_points.tolist())
        self.publisher.publish(cloud_msg)

    def _build_scene(self) -> np.ndarray:
        """Create a synthetic world holding only a cone pointing toward the camera."""

        points = self._sample_cone(self.points_per_frame)

        # Clip to max sensor range.
        dists = np.linalg.norm(points[:, :2], axis=1)
        within_range = dists <= self.max_range
        return points[within_range]

    def _sample_cone(self, count: int) -> np.ndarray:
        if count <= 0:
            return np.empty((0, 3))

        # Axis aligned with +X, tip closest to the camera at +X = cone_tip_distance.
        t = self.rng.uniform(0.0, 1.0, size=count)  # 0 at tip, 1 at base.
        x = self.cone_tip_distance + t * self.cone_length

        base_radius = self.cone_base_radius * np.maximum(t, 1e-3)
        theta = self.rng.uniform(0.0, 2.0 * math.pi, size=count)
        radial_scale = np.sqrt(self.rng.uniform(0.0, 1.0, size=count))  # Uniform disc fill.
        r = base_radius * radial_scale

        y = r * np.cos(theta)
        z = r * np.sin(theta)
        return np.stack((x, y, z), axis=1)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPointCloudPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
