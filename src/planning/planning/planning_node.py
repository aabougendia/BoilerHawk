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
        self._last_replan_time = None    # cooldown to avoid rapid replans

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
        """Generate initial global path as a straight line in world coords.

        A straight line is used because the occupancy grid is a small window
        centred on the drone and may not contain the far-away goal for A*.
        Obstacle avoidance is handled by local replanning in planning_callback.
        """
        self.global_path_world = self._straight_line_path(
            self.start_x, self.start_y, self.goal_x, self.goal_y)

        if self.global_path_world:
            self._publish_world_path(
                self.global_path_world, self.global_path_pub, 'map')
            self.get_logger().info(
                f'Global path (straight line): '
                f'{len(self.global_path_world)} waypoints  '
                f'({self.global_path_world[0]} → {self.global_path_world[-1]})')
        else:
            self.get_logger().error('Failed to compute global path')

    # ------------------------------------------------------------------
    # Helpers for straight-line / grid checking
    # ------------------------------------------------------------------
    @staticmethod
    def _straight_line_path(sx, sy, gx, gy, step=0.2):
        """Return list of (x, y) waypoints along a straight line."""
        dist = math.hypot(gx - sx, gy - sy)
        if dist < 0.01:
            return [(gx, gy)]
        n_steps = max(1, int(dist / step))
        return [
            (sx + (i / n_steps) * (gx - sx),
             sy + (i / n_steps) * (gy - sy))
            for i in range(n_steps + 1)
        ]

    def _is_in_grid(self, wx, wy):
        """Return True if (wx, wy) falls inside the current occupancy grid."""
        pp = self.path_planner
        if pp.grid_width == 0 or pp.grid_height == 0:
            return False
        col = round((wx - pp.grid_origin[0]) / pp.grid_resolution)
        row = round((wy - pp.grid_origin[1]) / pp.grid_resolution)
        return 0 <= row < pp.grid_height and 0 <= col < pp.grid_width

    def _replan_around_obstacle(self, closest_idx):
        """Use A* on the local grid to detour, stitch with remaining path."""
        sx = self.current_world_pos[0] if self.current_world_pos else self.start_x
        sy = self.current_world_pos[1] if self.current_world_pos else self.start_y

        # Find the first CLEAR, in-grid waypoint past the detected obstacle.
        # This avoids A*-ing toward a goal inside the obstacle.
        stitch_idx = len(self.global_path_world)
        obstacle_seen = False
        for i in range(closest_idx, len(self.global_path_world)):
            wx, wy = self.global_path_world[i]
            if not self._is_in_grid(wx, wy):
                stitch_idx = i
                break
            grid_cell = self.path_planner.world_to_grid((wx, wy))
            if not self.path_planner.is_valid_cell(grid_cell):
                obstacle_seen = True
            elif obstacle_seen:
                # First clear cell after the obstacle — good stitch point
                stitch_idx = i
                break

        # A* target: stitch waypoint, or goal if we ran off the path
        if stitch_idx < len(self.global_path_world):
            target = self.global_path_world[stitch_idx]
        else:
            target = (self.goal_x, self.goal_y)

        start_grid = self.path_planner.world_to_grid((sx, sy))
        goal_grid = self.path_planner.world_to_grid(target)

        grid_path = self.path_planner.plan_global_path(start_grid, goal_grid)
        if grid_path:
            astar_world = [
                self.path_planner.grid_to_world(gp) for gp in grid_path]
            # Stitch: A* detour + preserved waypoints beyond the obstacle
            remaining = self.global_path_world[stitch_idx:]
            self.global_path_world = astar_world + remaining
            self.get_logger().info(
                f'Replanned (A*): {len(self.global_path_world)} waypoints')
        else:
            # A* failed — keep current path
            self.get_logger().warn(
                'A* replan failed — keeping current path')

        self._publish_world_path(
            self.global_path_world, self.global_path_pub, 'map')
        self._last_replan_time = self.get_clock().now()

    # ------------------------------------------------------------------
    def planning_callback(self):
        """Periodic: validate global path, extract local window, publish."""
        if not self.occupancy_grid_received or not self.global_path_world:
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

        # --- Check ONLY waypoints AHEAD of the drone for obstacles ---
        # Ignore obstacles behind us — the drone already passed them.
        obstacle_found = False
        look_end = min(closest_idx + self.lookahead_distance,
                       len(self.global_path_world))
        for i in range(closest_idx, look_end):
            wp = self.global_path_world[i]
            if not self._is_in_grid(wp[0], wp[1]):
                continue
            grid_cell = self.path_planner.world_to_grid(wp)
            if not self.path_planner.is_valid_cell(grid_cell):
                obstacle_found = True
                break

        if obstacle_found:
            # Cooldown: don't replan more than once every 5 seconds
            now = self.get_clock().now()
            if (self._last_replan_time is not None
                    and (now - self._last_replan_time).nanoseconds
                    < 5_000_000_000):
                pass  # skip — too soon
            else:
                self.get_logger().warn('Obstacle ahead — replanning')
                self._replan_around_obstacle(closest_idx)
                if not self.global_path_world:
                    return
                # Recompute closest_idx on the new path
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
