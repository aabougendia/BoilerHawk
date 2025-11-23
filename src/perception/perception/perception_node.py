#!/usr/bin/env python3
"""
Perception Node for ROS 2.
Subscribes to occupancy grid from planning module and publishes local/global paths.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

from perception.path_planner import PathPlanner


class PerceptionNode(Node):
    """
    ROS 2 node for perception and path planning.
    """
    
    def __init__(self):
        super().__init__('perception_node')
        
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
        
        # Initialize path planner
        self.path_planner = PathPlanner(occupancy_threshold=occupancy_threshold)
        
        # State variables
        self.occupancy_grid_received = False
        self.global_path_computed = False
        self.current_position = None
        
        # Subscribers
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.occupancy_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
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
        
        self.get_logger().info('Perception node initialized')
        self.get_logger().info(f'Start: ({self.start_x}, {self.start_y}), Goal: ({self.goal_x}, {self.goal_y})')
    
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
        
        # Update path planner
        self.path_planner.update_occupancy_grid(
            grid_data,
            resolution,
            (origin_x, origin_y)
        )
        
        self.occupancy_grid_received = True
        self.get_logger().info(f'Occupancy grid received: {width}x{height}, resolution: {resolution}')
        
        # Trigger global path planning if not done yet
        if not self.global_path_computed:
            self.compute_global_path()
    
    def pose_callback(self, msg: PoseStamped):
        """
        Callback for current pose updates.
        
        Args:
            msg: PoseStamped message
        """
        world_pos = (msg.pose.position.x, msg.pose.position.y)
        self.current_position = self.path_planner.world_to_grid(world_pos)
    
    def compute_global_path(self):
        """
        Compute global path from start to goal.
        """
        if not self.occupancy_grid_received:
            self.get_logger().warn('Cannot compute global path: No occupancy grid received')
            return
        
        # Convert start and goal to grid coordinates
        start_grid = self.path_planner.world_to_grid((self.start_x, self.start_y))
        goal_grid = self.path_planner.world_to_grid((self.goal_x, self.goal_y))
        
        self.get_logger().info(f'Computing global path from {start_grid} to {goal_grid}')
        
        # Plan global path
        global_path = self.path_planner.plan_global_path(start_grid, goal_grid)
        
        if global_path:
            self.global_path_computed = True
            self.get_logger().info(f'Global path computed: {len(global_path)} waypoints')
            
            # Publish global path
            self.publish_path(global_path, self.global_path_pub, 'map')
        else:
            self.get_logger().error('Failed to compute global path')
    
    def planning_callback(self):
        """
        Periodic callback for path planning updates.
        """
        if not self.occupancy_grid_received:
            return
        
        if not self.global_path_computed:
            return
        
        # Use start position if current position not available
        if self.current_position is None:
            self.current_position = self.path_planner.world_to_grid((self.start_x, self.start_y))
        
        # Compute local path
        local_path = self.path_planner.plan_local_path(
            self.current_position,
            self.lookahead_distance
        )
        
        if local_path:
            # Publish local path
            self.publish_path(local_path, self.local_path_pub, 'map')
            
            # Publish visualization markers
            self.publish_path_markers()
    
    def publish_path(self, path: list, publisher, frame_id: str):
        """
        Publish path as ROS Path message.
        
        Args:
            path: List of grid positions
            publisher: ROS publisher
            frame_id: Frame ID for the path
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id
        
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
            global_marker.header.frame_id = 'map'
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
            local_marker.header.frame_id = 'map'
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
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
