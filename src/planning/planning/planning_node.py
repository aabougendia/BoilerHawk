#!/usr/bin/env python3
"""
Planning Node for BoilerHawk.
Subscribes to occupancy grid from perception, runs A* path planning,
and publishes nav_msgs/Path for the control node.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

from planning.path_planner import PathPlanner


class PlanningNode(Node):
    """
    ROS 2 node that bridges perception (occupancy grid) to control (path).
    Subscribes to /perception/occupancy and publishes /global_path and /local_path.
    """

    def __init__(self):
        super().__init__('planning_node')

        # --- Parameters ---
        self.declare_parameter('occupancy_topic', '/perception/occupancy')
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('local_path_topic', '/local_path')
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('lookahead_distance', 20)
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)
        self.declare_parameter('path_frame', 'map')

        self.occupancy_topic = str(self.get_parameter('occupancy_topic').value)
        self.global_path_topic = str(self.get_parameter('global_path_topic').value)
        self.local_path_topic = str(self.get_parameter('local_path_topic').value)
        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.lookahead_distance = int(self.get_parameter('lookahead_distance').value)
        self.planning_frequency = float(self.get_parameter('planning_frequency').value)
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.path_frame = str(self.get_parameter('path_frame').value)

        # --- Path planner ---
        self.planner = PathPlanner(occupancy_threshold=self.occupancy_threshold)

        # --- State ---
        self.latest_grid = None
        self.grid_info = None
        self.needs_replan = True
        self._last_log_time = None

        # --- Subscribers ---
        self.grid_sub = self.create_subscription(
            OccupancyGrid,
            self.occupancy_topic,
            self.occupancy_callback,
            10
        )

        # --- Publishers ---
        self.global_path_pub = self.create_publisher(Path, self.global_path_topic, 10)
        self.local_path_pub = self.create_publisher(Path, self.local_path_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

        # --- Timer for periodic replanning ---
        self.plan_timer = self.create_timer(
            1.0 / self.planning_frequency,
            self.planning_callback
        )

        self.get_logger().info(
            f'🗺️  Planning node started. '
            f'Subscribing to {self.occupancy_topic}, '
            f'publishing {self.global_path_topic} and {self.local_path_topic}'
        )
        self.get_logger().info(
            f'Goal: ({self.goal_x}, {self.goal_y}) | '
            f'Start: ({self.start_x}, {self.start_y}) | '
            f'Threshold: {self.occupancy_threshold}'
        )

    def occupancy_callback(self, msg: OccupancyGrid):
        """Store the latest occupancy grid and flag for replanning."""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Convert flat list to 2D numpy grid
        grid_data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        self.latest_grid = grid_data
        self.grid_info = {
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'frame_id': msg.header.frame_id,
        }
        self.needs_replan = True

        # Throttled logging
        now = self.get_clock().now()
        if (
            self._last_log_time is None
            or (now - self._last_log_time).nanoseconds >= 5_000_000_000
        ):
            occupied_count = int(np.sum(grid_data >= self.occupancy_threshold))
            self.get_logger().info(
                f'Received occupancy grid {width}x{height} '
                f'(res={resolution:.2f}m, occupied={occupied_count} cells)'
            )
            self._last_log_time = now

    def planning_callback(self):
        """Periodically run path planning if we have a grid."""
        if self.latest_grid is None or self.grid_info is None:
            return

        if not self.needs_replan:
            # Even if we don't need to replan, keep publishing the last path
            # so the control node always has something
            return

        info = self.grid_info
        resolution = info['resolution']
        origin = (info['origin_x'], info['origin_y'])

        # Update the planner's occupancy grid
        self.planner.update_occupancy_grid(self.latest_grid, resolution, origin)

        # Convert world start/goal to grid coordinates
        start_grid = self.planner.world_to_grid((self.start_x, self.start_y))
        goal_grid = self.planner.world_to_grid((self.goal_x, self.goal_y))

        # Clamp to valid grid bounds
        start_grid = (
            max(0, min(start_grid[0], info['height'] - 1)),
            max(0, min(start_grid[1], info['width'] - 1)),
        )
        goal_grid = (
            max(0, min(goal_grid[0], info['height'] - 1)),
            max(0, min(goal_grid[1], info['width'] - 1)),
        )

        self.get_logger().info(
            f'Planning: start_grid={start_grid}, goal_grid={goal_grid}'
        )

        # Run A* global path planning
        global_path_cells = self.planner.plan_global_path(start_grid, goal_grid)

        if not global_path_cells:
            self.get_logger().warn('No global path found!')
            self.needs_replan = False
            return

        # Convert grid path to world coordinates and publish as nav_msgs/Path
        global_path_msg = self._cells_to_path(global_path_cells)
        self.global_path_pub.publish(global_path_msg)

        # Plan local path (subset for control)
        local_path_cells = self.planner.plan_local_path(
            start_grid, lookahead_distance=self.lookahead_distance
        )
        if local_path_cells:
            local_path_msg = self._cells_to_path(local_path_cells)
        else:
            local_path_msg = global_path_msg  # fallback

        self.local_path_pub.publish(local_path_msg)

        # Publish visualization markers
        self._publish_markers(global_path_cells, local_path_cells)

        self.get_logger().info(
            f'Published paths: global={len(global_path_cells)} wp, '
            f'local={len(local_path_cells) if local_path_cells else 0} wp'
        )

        self.needs_replan = False

    def _cells_to_path(self, cells) -> Path:
        """Convert a list of grid cells to a nav_msgs/Path in world coords."""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.path_frame

        for cell in cells:
            world_x, world_y = self.planner.grid_to_world(cell)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(world_x)
            pose.pose.position.y = float(world_y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _publish_markers(self, global_cells, local_cells):
        """Publish RViz markers for the planned paths."""
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Global path line strip (green)
        global_marker = Marker()
        global_marker.header.stamp = stamp
        global_marker.header.frame_id = self.path_frame
        global_marker.ns = 'global_path'
        global_marker.id = 0
        global_marker.type = Marker.LINE_STRIP
        global_marker.action = Marker.ADD
        global_marker.scale.x = 0.05
        global_marker.color.r = 0.0
        global_marker.color.g = 1.0
        global_marker.color.b = 0.0
        global_marker.color.a = 0.8

        for cell in global_cells:
            wx, wy = self.planner.grid_to_world(cell)
            from geometry_msgs.msg import Point
            p = Point()
            p.x = float(wx)
            p.y = float(wy)
            p.z = 0.1
            global_marker.points.append(p)

        markers.markers.append(global_marker)

        # Local path line strip (red)
        if local_cells:
            local_marker = Marker()
            local_marker.header.stamp = stamp
            local_marker.header.frame_id = self.path_frame
            local_marker.ns = 'local_path'
            local_marker.id = 1
            local_marker.type = Marker.LINE_STRIP
            local_marker.action = Marker.ADD
            local_marker.scale.x = 0.08
            local_marker.color.r = 1.0
            local_marker.color.g = 0.0
            local_marker.color.b = 0.0
            local_marker.color.a = 0.9

            for cell in local_cells:
                wx, wy = self.planner.grid_to_world(cell)
                from geometry_msgs.msg import Point
                p = Point()
                p.x = float(wx)
                p.y = float(wy)
                p.z = 0.15
                local_marker.points.append(p)

            markers.markers.append(local_marker)

        # Start marker (blue sphere)
        start_marker = Marker()
        start_marker.header.stamp = stamp
        start_marker.header.frame_id = self.path_frame
        start_marker.ns = 'waypoints'
        start_marker.id = 2
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.3
        start_marker.color.r = 0.0
        start_marker.color.g = 0.0
        start_marker.color.b = 1.0
        start_marker.color.a = 1.0
        start_marker.pose.position.x = self.start_x
        start_marker.pose.position.y = self.start_y
        start_marker.pose.position.z = 0.15
        start_marker.pose.orientation.w = 1.0
        markers.markers.append(start_marker)

        # Goal marker (yellow sphere)
        goal_marker = Marker()
        goal_marker.header.stamp = stamp
        goal_marker.header.frame_id = self.path_frame
        goal_marker.ns = 'waypoints'
        goal_marker.id = 3
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.3
        goal_marker.color.r = 1.0
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        goal_marker.pose.position.x = self.goal_x
        goal_marker.pose.position.y = self.goal_y
        goal_marker.pose.position.z = 0.15
        goal_marker.pose.orientation.w = 1.0
        markers.markers.append(goal_marker)

        self.marker_pub.publish(markers)


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
