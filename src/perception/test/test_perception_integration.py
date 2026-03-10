#!/usr/bin/env python3
# Copyright 2026 BoilerHawk Team
# MIT License
"""
Integration test for the PerceptionNode.

Spins up the perception node together with a synthetic PointCloud2 publisher
and verifies that a valid OccupancyGrid is produced with the expected
properties (correct frame, resolution, occupied cells).

Run with:
    cd ~/BoilerHawk/BoilerHawk
    colcon test --packages-select perception
    colcon test-result --verbose
"""

import threading
import time
import unittest

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from perception.perception import PerceptionNode


class SyntheticSourceNode(Node):
    """Publishes a synthetic point cloud and drone pose for testing."""

    def __init__(self):
        super().__init__('test_synthetic_source')

        # Point cloud publisher (matches perception default topic)
        self.pc_pub = self.create_publisher(
            PointCloud2, '/test/points', 10
        )

        # Pose publisher (BEST_EFFORT to match what perception expects)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/test/pose',
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self._publish)
        self.get_logger().info('Synthetic source node started')

    def _publish(self):
        now = self.get_clock().now().to_msg()

        # --- Drone at origin, yaw = 0 ---
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        # --- Point cloud: a wall at x=2.0, spanning y=[-1, 1] ---
        n_points = 200
        xs = np.full(n_points, 2.0, dtype=np.float32)
        ys = np.linspace(-1.0, 1.0, n_points, dtype=np.float32)
        zs = np.zeros(n_points, dtype=np.float32)
        points = np.column_stack((xs, ys, zs))

        header = Header()
        header.stamp = now
        header.frame_id = 'camera_link'
        cloud = pc2.create_cloud_xyz32(header, points.tolist())
        self.pc_pub.publish(cloud)


class OccupancyGridCollector(Node):
    """Subscribes to the occupancy grid and stores received messages."""

    def __init__(self):
        super().__init__('test_grid_collector')
        self.grids = []
        self.create_subscription(
            OccupancyGrid, '/test/occupancy', self._cb, 10
        )

    def _cb(self, msg):
        self.grids.append(msg)


class TestPerceptionIntegration(unittest.TestCase):
    """Integration test: synthetic point cloud → PerceptionNode → OccupancyGrid."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_occupancy_grid_from_synthetic_cloud(self):
        """Verify perception produces a valid occupancy grid from synthetic data."""

        # Create nodes
        source = SyntheticSourceNode()
        collector = OccupancyGridCollector()

        # Create perception node with test-specific topic remapping
        perception = PerceptionNode()
        # Override parameters for test topics
        perception.pointcloud_topic = '/test/points'
        perception.occupancy_topic = '/test/occupancy'
        perception.pose_topic = '/test/pose'
        perception.resolution = 0.1
        perception.max_range = 5.0
        perception.grid_frame = 'map'

        # Recreate subscriptions and publisher with test topics
        perception.destroy_subscription(perception.subscription)
        perception.subscription = perception.create_subscription(
            PointCloud2, '/test/points',
            perception.pointcloud_callback, 10,
        )
        # Recreate pose subscription
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        perception.create_subscription(
            PoseStamped, '/test/pose',
            perception._pose_cb, mavros_qos,
        )
        perception.destroy_publisher(perception.map_pub)
        perception.map_pub = perception.create_publisher(
            OccupancyGrid, '/test/occupancy', 10,
        )

        executor = MultiThreadedExecutor()
        executor.add_node(source)
        executor.add_node(perception)
        executor.add_node(collector)

        # Spin for up to 5 seconds waiting for grids
        spin_thread = threading.Thread(
            target=executor.spin, daemon=True
        )
        spin_thread.start()

        deadline = time.time() + 5.0
        while time.time() < deadline and len(collector.grids) < 3:
            time.sleep(0.1)

        # Clean up nodes before shutting down executor to avoid
        # "destruction was requested" warnings.
        source.destroy_node()
        perception.destroy_node()
        collector.destroy_node()
        executor.shutdown()
        spin_thread.join(timeout=2.0)

        # --- Assertions ---
        self.assertGreaterEqual(
            len(collector.grids), 1,
            'No occupancy grids received from perception node'
        )

        grid_msg = collector.grids[-1]

        # Frame and resolution
        self.assertEqual(grid_msg.header.frame_id, 'map')
        self.assertAlmostEqual(grid_msg.info.resolution, 0.1, places=3)

        # Grid dimensions (max_range=5 → 2*5/0.1 = 100)
        self.assertEqual(grid_msg.info.width, 100)
        self.assertEqual(grid_msg.info.height, 100)

        # Origin should be at drone position minus max_range = (0-5, 0-5)
        self.assertAlmostEqual(grid_msg.info.origin.position.x, -5.0, places=1)
        self.assertAlmostEqual(grid_msg.info.origin.position.y, -5.0, places=1)

        # The grid should have SOME occupied cells (the wall at x=2)
        grid_data = np.array(grid_msg.data, dtype=np.int8)
        occupied_count = np.sum(grid_data == 100)
        self.assertGreater(
            occupied_count, 0,
            'Occupancy grid has no occupied cells — wall not detected'
        )

        # Occupied cells should be around x=2.0 relative to grid origin
        # x=2.0 in world → grid col = (2.0 - (-5.0)) / 0.1 = 70
        grid_2d = grid_data.reshape(
            grid_msg.info.height, grid_msg.info.width
        )
        # Check that column ~70 has occupied cells
        col_70_occupied = np.sum(grid_2d[:, 68:72] == 100)
        self.assertGreater(
            col_70_occupied, 0,
            'Wall at x=2.0 not found near expected grid column (~70)'
        )


if __name__ == '__main__':
    unittest.main()
