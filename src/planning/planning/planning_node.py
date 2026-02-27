#!/usr/bin/env python3
"""
Planning Node for ROS 2.
Subscribes to occupancy grid from perception and publishes global/local paths.

Key design: paths are stored in **world coordinates** so they remain stable
even though the drone-centred occupancy grid shifts every frame.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

from planning.path_planner import PathPlanner


class PlanningNode(Node):
    """ROS 2 node for A*-based path planning on a live occupancy grid."""

    def __init__(self):
        super().__init__('planning_node')

        # Declare parameters
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('lookahead_distance', 20)
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)

        # Get parameters
        occupancy_threshold = self.get_parameter('occupancy_threshold').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        planning_frequency = self.get_parameter('planning_frequency').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        # Initialize path planner (grid-level A*)
        self.path_planner = PathPlanner(occupancy_threshold=occupancy_threshold)

        # ---- State ----
        self.occupancy_grid_received = False
        # World-coordinate paths (list of (x, y) tuples)
        self.global_path_world = []      # full start→goal in world coords
        self.local_path_world = []       # upcoming segment for control
        self.current_world_pos = None    # (x, y) from /current_pose

        # ---- Subscribers ----
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid, '/occupancy_grid', self.occupancy_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self.pose_callback, 10)

        # ---- Publishers ----
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.path_markers_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

        # Timer for periodic local-path updates
        self.create_timer(1.0 / planning_frequency, self.planning_callback)

        self.get_logger().info('Planning node initialized')
        self.get_logger().info(
            f'Start: ({self.start_x}, {self.start_y}), '
            f'Goal: ({self.goal_x}, {self.goal_y})')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def occupancy_callback(self, msg: OccupancyGrid):
        """Update internal grid; compute global path if not yet available."""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        grid_data = np.array(msg.data).reshape((height, width))
        self.path_planner.update_occupancy_grid(
            grid_data, resolution, (origin_x, origin_y))

        self.occupancy_grid_received = True

        # Compute global path only once (will replan if obstacles invalidate it)
        if not self.global_path_world:
            self._compute_global_path()

    def pose_callback(self, msg: PoseStamped):
        """Store current drone position in world coordinates."""
        self.current_world_pos = (msg.pose.position.x, msg.pose.position.y)

    # ------------------------------------------------------------------
    # Path computation
    # ------------------------------------------------------------------
    def _compute_global_path(self):
        """Run A* on the current grid and store the result in world coords."""
        if not self.occupancy_grid_received:
            return

        start_grid = self.path_planner.world_to_grid((self.start_x, self.start_y))
        goal_grid = self.path_planner.world_to_grid((self.goal_x, self.goal_y))

        self.get_logger().info(
            f'Computing global path from {start_grid} to {goal_grid}')

        grid_path = self.path_planner.plan_global_path(start_grid, goal_grid)
        if grid_path:
            # Convert to world coordinates immediately and store
            self.global_path_world = [
                self.path_planner.grid_to_world(gp) for gp in grid_path]
            self.get_logger().info(
                f'Global path computed: {len(self.global_path_world)} waypoints  '
                f'({self.global_path_world[0]} → {self.global_path_world[-1]})')
            self._publish_world_path(
                self.global_path_world, self.global_path_pub, 'map')
        else:
            self.get_logger().error('Failed to compute global path')

    # ------------------------------------------------------------------
    def planning_callback(self):
        """Periodic: validate global path, extract local window, publish."""
        if not self.occupancy_grid_received or not self.global_path_world:
            return

        # --- Check for obstacles on global path (replan if needed) ---
        obstacle_found = False
        for wp in self.global_path_world:
            grid_cell = self.path_planner.world_to_grid(wp)
            if not self.path_planner.is_valid_cell(grid_cell):
                obstacle_found = True
                break

        if obstacle_found:
            self.get_logger().warn('Obstacle on global path — replanning')
            self.global_path_world = []
            # Replan from current position (or start if no pose yet)
            if self.current_world_pos is not None:
                self.start_x, self.start_y = self.current_world_pos
            self._compute_global_path()
            if not self.global_path_world:
                return

        # --- Determine current position ---
        if self.current_world_pos is not None:
            cx, cy = self.current_world_pos
        else:
            cx, cy = self.start_x, self.start_y

        # --- Find closest waypoint on global path ---
        min_dist = float('inf')
        closest_idx = 0
        for idx, (wx, wy) in enumerate(self.global_path_world):
            d = math.hypot(wx - cx, wy - cy)
            if d < min_dist:
                min_dist = d
                closest_idx = idx

        # --- Extract local path window ---
        end_idx = min(closest_idx + self.lookahead_distance,
                      len(self.global_path_world))
        self.local_path_world = self.global_path_world[closest_idx:end_idx]

        if self.local_path_world:
            self._publish_world_path(
                self.local_path_world, self.local_path_pub, 'map')
            self._publish_path_markers()

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------
    def _publish_world_path(self, world_path, publisher, frame_id):
        """Publish a list of (x, y) world-coord tuples as a nav_msgs/Path."""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for wx, wy in world_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(wx)
            pose.pose.position.y = float(wy)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        publisher.publish(path_msg)

    def _publish_path_markers(self):
        """Publish RViz visualisation markers for global & local paths."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Global path – blue
        if self.global_path_world:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'global_path'
            m.id = 0
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.b = 1.0
            m.color.a = 0.8
            for wx, wy in self.global_path_world:
                pt = PoseStamped().pose.position
                pt.x = float(wx)
                pt.y = float(wy)
                pt.z = 0.0
                m.points.append(pt)
            marker_array.markers.append(m)

        # Local path – red
        if self.local_path_world:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'local_path'
            m.id = 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.08
            m.color.r = 1.0
            m.color.a = 1.0
            for wx, wy in self.local_path_world:
                pt = PoseStamped().pose.position
                pt.x = float(wx)
                pt.y = float(wy)
                pt.z = 0.0
                m.points.append(pt)
            marker_array.markers.append(m)

        self.path_markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
