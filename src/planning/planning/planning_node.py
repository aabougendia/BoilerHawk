#!/usr/bin/env python3
"""
Planning Node for ROS 2.
Subscribes to occupancy grid from perception module and publishes local/global paths.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

from planning.path_planner import PathPlanner, VFHPlanner


def _quaternion_to_yaw(qx, qy, qz, qw):
    """Extract yaw (heading) from quaternion; returns radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PlanningNode(Node):
    """
    ROS 2 node for planning and path planning.
    """
    
    def __init__(self):
        super().__init__('planning_node')
        
        # Declare parameters
        self.declare_parameter('occupancy_topic', '/occupancy_grid')
        self.declare_parameter('pose_topic', '/current_pose')
        self.declare_parameter('path_frame', 'map')
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('lookahead_distance', 20)
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 5.0)
        self.declare_parameter('avoid_only', False)
        self.declare_parameter('goal_ahead_distance', 5.0)
        self.declare_parameter('use_robot_centric_grid', True)

        # VFH parameters
        self.declare_parameter('safety_radius_cells', 3)
        self.declare_parameter('num_sectors', 72)
        self.declare_parameter('valley_threshold', 0.5)
        self.declare_parameter('num_waypoints', 3)
        self.declare_parameter('waypoint_spacing', 1.0)
        self.declare_parameter('goal_reached_threshold', 1.0)
        self.declare_parameter('heading_ema_alpha', 0.3)  # low-pass: 0.3*new + 0.7*prev

        # Get parameters
        occupancy_topic = self.get_parameter('occupancy_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.path_frame = self.get_parameter('path_frame').value
        occupancy_threshold = self.get_parameter('occupancy_threshold').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        planning_frequency = self.get_parameter('planning_frequency').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.avoid_only = self.get_parameter('avoid_only').value
        self.goal_ahead_distance = self.get_parameter('goal_ahead_distance').value
        self.use_robot_centric_grid = self.get_parameter('use_robot_centric_grid').value

        safety_radius_cells = self.get_parameter('safety_radius_cells').value
        num_sectors = self.get_parameter('num_sectors').value
        valley_threshold = self.get_parameter('valley_threshold').value
        num_waypoints = self.get_parameter('num_waypoints').value
        waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.goal_reached_threshold = self.get_parameter('goal_reached_threshold').value
        self.heading_ema_alpha = self.get_parameter('heading_ema_alpha').value

        # Initialize path planners
        self.path_planner = PathPlanner(occupancy_threshold=occupancy_threshold)
        self.vfh_planner = VFHPlanner(
            occupancy_threshold=occupancy_threshold,
            safety_radius_cells=safety_radius_cells,
            num_sectors=num_sectors,
            valley_threshold=valley_threshold,
            num_waypoints=num_waypoints,
            waypoint_spacing=waypoint_spacing,
        )

        # State variables
        self.occupancy_grid_received = False
        self.global_path_computed = False
        self.current_position = None

        # VFH smoothing / hysteresis state
        self._last_vfh_path = None    # fallback when VFH returns empty
        self._last_chosen_angle = None  # for heading rate-limiting

        # Subscribers (configurable for real perception / localization)
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            occupancy_topic,
            self.occupancy_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            10
        )
        
        # Publishers
        self.global_path_pub = self.create_publisher(
            Path,
            '/global_path',
            10
        )
        
        self.local_path_pub = self.create_publisher(
            Path,
            '/local_path',
            10
        )
        
        self.path_markers_pub = self.create_publisher(
            MarkerArray,
            '/path_markers',
            10
        )
        
        # Timer for periodic path updates
        self.create_timer(1.0 / planning_frequency, self.planning_callback)
        
        # World pose from localization (used for dynamic start when grid available)
        self.current_pose_world = None
        self.current_pose_msg = None  # full pose for yaw in avoid_only

        self.get_logger().info('Planning node initialized')
        self.get_logger().info(
            f'Occupancy: {occupancy_topic}, Pose: {pose_topic}, Frame: {self.path_frame}'
        )
        if self.avoid_only:
            self.get_logger().info(
                f'Avoid-only mode: goal {self.goal_ahead_distance}m ahead, '
                f'robot_centric={self.use_robot_centric_grid}'
            )
        else:
            self.get_logger().info(f'Goal (params): ({self.goal_x}, {self.goal_y})')
    
    def occupancy_callback(self, msg: OccupancyGrid):
        """
        Callback for occupancy grid updates.
        
        Args:
            msg: OccupancyGrid message
        """
        # Convert occupancy grid to numpy array
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        # Reshape data to 2D grid
        grid_data = np.array(msg.data).reshape((height, width))
        
        # Update path planners
        self.path_planner.update_occupancy_grid(
            grid_data,
            resolution,
            (origin_x, origin_y)
        )
        self.vfh_planner.update_occupancy_grid(
            grid_data,
            resolution,
            (origin_x, origin_y)
        )
        
        self.occupancy_grid_received = True
        self.get_logger().info(f'Occupancy grid received: {width}x{height}, resolution: {resolution}')
        
        # In avoid_only we recompute every cycle; otherwise compute once
        if not self.avoid_only and not self.global_path_computed:
            self.compute_global_path()

    def pose_callback(self, msg: PoseStamped):
        """
        Callback for current pose updates (localization / drone pose).
        Updates current position for local planning and dynamic start.
        """
        self.current_pose_msg = msg
        self.current_pose_world = (msg.pose.position.x, msg.pose.position.y)
        if self.path_planner.occupancy_grid is not None:
            self.current_position = self.path_planner.world_to_grid(self.current_pose_world)
        if self.occupancy_grid_received and not self.global_path_computed and not self.avoid_only:
            self.compute_global_path()
    
    def _grid_path_to_world_path(self, path: list) -> list:
        """
        Convert a path in grid coords to world coords when grid is robot-centric.
        Perception frame (grid origin at drone) -> map/local frame using current pose + yaw.
        """
        if not path or self.current_pose_world is None or self.current_pose_msg is None:
            return []
        q = self.current_pose_msg.pose.orientation
        yaw = _quaternion_to_yaw(q.x, q.y, q.z, q.w)
        cx, cy = self.current_pose_world
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        planner = self.vfh_planner if self.use_robot_centric_grid else self.path_planner
        out = []
        for grid_pos in path:
            px, py = planner.grid_to_world(grid_pos)
            wx = cx + cos_yaw * px - sin_yaw * py
            wy = cy + sin_yaw * px + cos_yaw * py
            out.append((wx, wy))
        return out

    def _bearing_to_goal_in_robot_frame(self) -> float:
        """Angle from drone to goal_x/goal_y, relative to drone heading (robot frame)."""
        if self.current_pose_world is None or self.current_pose_msg is None:
            return 0.0
        dx = self.goal_x - self.current_pose_world[0]
        dy = self.goal_y - self.current_pose_world[1]
        world_bearing = math.atan2(dy, dx)
        q = self.current_pose_msg.pose.orientation
        yaw = _quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return world_bearing - yaw

    def _goal_reached(self) -> bool:
        """True when the drone is within *goal_reached_threshold* of the goal."""
        if self.avoid_only or self.current_pose_world is None:
            return False
        dx = self.goal_x - self.current_pose_world[0]
        dy = self.goal_y - self.current_pose_world[1]
        return math.hypot(dx, dy) < self.goal_reached_threshold

    def compute_global_path(self):
        """
        Compute global path from start to goal.
        Uses current drone pose as start when available (dynamic start); otherwise params.
        In avoid_only mode goal is a point ahead of the drone (no fixed destination).
        """
        if not self.occupancy_grid_received:
            self.get_logger().warn('Cannot compute global path: No occupancy grid received')
            return

        if self.current_pose_world is not None:
            start_grid = self.path_planner.world_to_grid(self.current_pose_world)
        else:
            start_grid = self.path_planner.world_to_grid((self.start_x, self.start_y))
        goal_grid = self.path_planner.world_to_grid((self.goal_x, self.goal_y))

        global_path = self.path_planner.plan_global_path(start_grid, goal_grid)

        if global_path:
            self.global_path_computed = True
            self.publish_path(global_path, self.global_path_pub, self.path_frame)
        else:
            self.get_logger().error('Failed to compute global path')
    
    def planning_callback(self):
        """
        Periodic callback for path planning updates.

        Robot-centric mode (VFH): reactive avoidance, always produces waypoints.
        Non-robot-centric mode (A*): global + local planning, kept for compatibility.
        """
        if not self.occupancy_grid_received:
            return

        # ---------- VFH robot-centric path (avoid-only OR goal mode) ----------
        if self.use_robot_centric_grid:
            if not self.avoid_only and self._goal_reached():
                self.get_logger().info('Goal reached — hovering')
                return

            if self.avoid_only:
                preferred_angle = 0.0
            else:
                preferred_angle = self._bearing_to_goal_in_robot_frame()

            path, chosen_angle = self.vfh_planner.plan(preferred_angle)

            # --- EMA low-pass filter on heading (prevents oscillation) ---
            if self._last_chosen_angle is not None and path:
                # Compute angular difference properly (handle wrap-around)
                diff = (chosen_angle - self._last_chosen_angle + math.pi) % (2 * math.pi) - math.pi
                # EMA blend: alpha * new + (1 - alpha) * prev
                filtered = self._last_chosen_angle + self.heading_ema_alpha * diff
                if abs(diff) > 0.05:  # only log when there's meaningful change
                    self.get_logger().info(
                        f'[VFH] EMA filter: raw={math.degrees(chosen_angle):.0f}° '
                        f'filtered={math.degrees(filtered):.0f}°')
                # Recompute waypoints at the filtered angle
                path, chosen_angle = self.vfh_planner.plan(
                    filtered,
                    num_waypoints=self.vfh_planner.num_waypoints,
                    spacing_m=self.vfh_planner.waypoint_spacing)

            # --- fallback: never publish an empty path ---
            is_fallback = False
            if not path:
                if self._last_vfh_path:
                    path = self._last_vfh_path
                    is_fallback = True
                    self.get_logger().warn('[VFH] Empty path — reusing last successful path')
                else:
                    self.get_logger().warn('[VFH] No path and no fallback available')
                    return

            # --- diagnostics ---
            self.get_logger().info(
                f'[VFH] pref={math.degrees(preferred_angle):.0f}° '
                f'chosen={math.degrees(chosen_angle):.0f}° '
                f'wps={len(path)} fallback={is_fallback}')

            # --- update state and publish ---
            self._last_vfh_path = path
            self._last_chosen_angle = chosen_angle

            if self.current_pose_world and self.current_pose_msg:
                self.publish_path(path, self.local_path_pub, self.path_frame)
            return

        # ---------- Legacy A*-based planning (non-robot-centric) ----------
        if self.current_position is None and self.current_pose_world is not None:
            self.current_position = self.path_planner.world_to_grid(self.current_pose_world)
        if self.current_position is None:
            self.current_position = self.path_planner.world_to_grid((self.start_x, self.start_y))

        if not self.global_path_computed:
            return

        local_path = self.path_planner.plan_local_path(
            self.current_position,
            self.lookahead_distance
        )

        if local_path:
            self.publish_path(local_path, self.local_path_pub, self.path_frame)
            self.publish_path_markers()
    
    def publish_path(self, path: list, publisher, frame_id: str):
        """
        Publish path as ROS Path message.
        When use_robot_centric_grid, path is in grid coords; transform to world using pose + yaw.
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id or self.path_frame

        if self.use_robot_centric_grid and self.current_pose_world and self.current_pose_msg:
            world_points = self._grid_path_to_world_path(path)
            for wx, wy in world_points:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wx
                pose.pose.position.y = wy
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
        else:
            for grid_pos in path:
                world_pos = self.path_planner.grid_to_world(grid_pos)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = world_pos[0]
                pose.pose.position.y = world_pos[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

        publisher.publish(path_msg)
    
    def publish_path_markers(self):
        """
        Publish visualization markers for paths.
        """
        marker_array = MarkerArray()
        
        # Global path marker
        if self.path_planner.global_path:
            global_marker = Marker()
            global_marker.header.frame_id = self.path_frame
            global_marker.header.stamp = self.get_clock().now().to_msg()
            global_marker.ns = 'global_path'
            global_marker.id = 0
            global_marker.type = Marker.LINE_STRIP
            global_marker.action = Marker.ADD
            global_marker.scale.x = 0.05
            global_marker.color.r = 0.0
            global_marker.color.g = 0.0
            global_marker.color.b = 1.0
            global_marker.color.a = 0.8
            
            for grid_pos in self.path_planner.global_path:
                world_pos = self.path_planner.grid_to_world(grid_pos)
                point = PoseStamped().pose.position
                point.x = world_pos[0]
                point.y = world_pos[1]
                point.z = 0.0
                global_marker.points.append(point)
            
            marker_array.markers.append(global_marker)
        
        # Local path marker
        if self.path_planner.local_path:
            local_marker = Marker()
            local_marker.header.frame_id = self.path_frame
            local_marker.header.stamp = self.get_clock().now().to_msg()
            local_marker.ns = 'local_path'
            local_marker.id = 1
            local_marker.type = Marker.LINE_STRIP
            local_marker.action = Marker.ADD
            local_marker.scale.x = 0.08
            local_marker.color.r = 1.0
            local_marker.color.g = 0.0
            local_marker.color.b = 0.0
            local_marker.color.a = 1.0
            
            for grid_pos in self.path_planner.local_path:
                world_pos = self.path_planner.grid_to_world(grid_pos)
                point = PoseStamped().pose.position
                point.x = world_pos[0]
                point.y = world_pos[1]
                point.z = 0.0
                local_marker.points.append(point)
            
            marker_array.markers.append(local_marker)
        
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
