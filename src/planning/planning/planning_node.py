#!/usr/bin/env python3
"""
Planning Node — A*-based path planning on a persistent occupancy grid.

Algorithm (simple & robust):
  1. Grid starts all-free → initial A* gives a straight-ish path to goal.
  2. As the drone flies, perception marks obstacles in the grid.
  3. Each planning tick: check if any waypoint ahead is now occupied.
  4. If so, recompute A* from the drone's current position to the goal.
  5. Publish global path + local lookahead window for control.
"""

import logging
import math
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

from planning.path_planner import PathPlanner


def _setup_file_logger(node_name: str) -> logging.Logger:
    log_dir = os.path.expanduser('~/BoilerHawk/BoilerHawk/logs')
    os.makedirs(log_dir, exist_ok=True)
    flog = logging.getLogger(f'bhawk.{node_name}')
    flog.setLevel(logging.DEBUG)
    if not flog.handlers:
        fh = logging.FileHandler(
            os.path.join(log_dir, f'{node_name}.log'), mode='w')
        fh.setFormatter(logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%H:%M:%S'))
        flog.addHandler(fh)
    return flog


class PlanningNode(Node):
    """ROS 2 node for A*-based path planning on a live occupancy grid."""

    def __init__(self):
        super().__init__('planning_node')

        # Parameters
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('lookahead_distance', 30)
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 10.0)

        self.lookahead = self.get_parameter('lookahead_distance').value
        freq = self.get_parameter('planning_frequency').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.planner = PathPlanner(
            occupancy_threshold=self.get_parameter('occupancy_threshold').value)

        # File logger
        self.flog = _setup_file_logger('planning')
        self.flog.info('=== Planning node started ===')

        # State
        self.global_path = []      # [(x, y), ...] in world coords
        self.current_pos = None    # (x, y)
        self.grid_ready = False
        self._replan_cooldown_ns = 2_000_000_000  # 2 s
        self._last_replan_time = None

        # Subscribers
        self.create_subscription(
            OccupancyGrid, '/occupancy_grid', self._grid_cb, 10)
        self.create_subscription(
            PoseStamped, '/current_pose', self._pose_cb, 10)

        # Publishers
        self.global_pub = self.create_publisher(Path, '/global_path', 10)
        self.local_pub = self.create_publisher(Path, '/local_path', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/path_markers', 10)

        self.create_timer(1.0 / freq, self._tick)

        plan_msg = (
            f'Planning: ({self.start_x},{self.start_y}) → '
            f'({self.goal_x},{self.goal_y})')
        self.get_logger().info(plan_msg)
        self.flog.info(plan_msg)

    # ---- Callbacks ------------------------------------------------

    def _pose_cb(self, msg: PoseStamped):
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)

    def _grid_cb(self, msg: OccupancyGrid):
        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.planner.update_occupancy_grid(
            grid, msg.info.resolution,
            (msg.info.origin.position.x, msg.info.origin.position.y))
        self.grid_ready = True

        occ_pct = 100.0 * np.count_nonzero(grid >= 50) / grid.size
        self.flog.info(
            f'Grid update: {msg.info.width}x{msg.info.height}, '
            f'{occ_pct:.1f}% occupied')

        # First grid → compute initial path
        if not self.global_path:
            self._plan_from(self.start_x, self.start_y)

    # ---- Planning -------------------------------------------------

    def _plan_from(self, sx, sy):
        """Run A* from (sx, sy) to goal; update global_path."""
        s = self.planner.world_to_grid((sx, sy))
        g = self.planner.world_to_grid((self.goal_x, self.goal_y))
        result = self.planner.plan_global_path(s, g)
        if result:
            self.global_path = [
                self.planner.grid_to_world(c) for c in result]
            self._publish(self.global_path, self.global_pub)
            path_msg = f'Path: {len(self.global_path)} waypoints from ({sx:.1f},{sy:.1f})'
            self.get_logger().info(path_msg)
            self.flog.info(path_msg)
        else:
            self.get_logger().warn('A* failed — keeping previous path')
            self.flog.warning(f'A* FAILED from ({sx:.1f},{sy:.1f}) to ({self.goal_x:.1f},{self.goal_y:.1f})')

    def _tick(self):
        """Periodic: check path validity, replan if blocked, publish."""
        if not self.grid_ready or not self.global_path:
            return

        cx, cy = self.current_pos or (self.start_x, self.start_y)

        # Find closest waypoint
        closest = min(
            range(len(self.global_path)),
            key=lambda i: math.hypot(
                self.global_path[i][0] - cx,
                self.global_path[i][1] - cy))

        # Check waypoints AHEAD for obstacles
        blocked = False
        for wx, wy in self.global_path[closest:]:
            cell = self.planner.world_to_grid((wx, wy))
            if not self.planner.is_valid_cell(cell):
                blocked = True
                break

        if blocked:
            now = self.get_clock().now()
            ok = (self._last_replan_time is None
                  or (now - self._last_replan_time).nanoseconds
                  >= self._replan_cooldown_ns)
            if ok:
                self.get_logger().warn('Obstacle on path — replanning')
                self.flog.warning(f'Replan triggered at ({cx:.1f},{cy:.1f})')
                self._plan_from(cx, cy)
                self._last_replan_time = now
                # Recompute closest after replan
                if self.global_path:
                    closest = min(
                        range(len(self.global_path)),
                        key=lambda i: math.hypot(
                            self.global_path[i][0] - cx,
                            self.global_path[i][1] - cy))

        # Local path = upcoming waypoint window
        end = min(closest + self.lookahead, len(self.global_path))
        local = self.global_path[closest:end]
        if local:
            self._publish(local, self.local_pub)
        self._publish_markers()

    # ---- Publishing -----------------------------------------------

    def _publish(self, path, pub):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wx, wy in path:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = float(wx)
            p.pose.position.y = float(wy)
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        pub.publish(msg)

    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        if self.global_path:
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
            for wx, wy in self.global_path:
                pt = PoseStamped().pose.position
                pt.x, pt.y, pt.z = float(wx), float(wy), 0.1
                m.points.append(pt)
            ma.markers.append(m)
        self.marker_pub.publish(ma)


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
