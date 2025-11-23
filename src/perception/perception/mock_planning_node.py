#!/usr/bin/env python3
"""
Mock Planning Node for testing perception module.
Generates and publishes hypothetical occupancy grids.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header
import numpy as np


class MockPlanningNode(Node):
    """
    Mock planning node that publishes test occupancy grids.
    """
    
    def __init__(self):
        super().__init__('mock_planning_node')
        
        # Declare parameters
        self.declare_parameter('grid_width', 100)
        self.declare_parameter('grid_height', 100)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('add_dynamic_obstacles', False)
        
        # Get parameters
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        publish_frequency = self.get_parameter('publish_frequency').value
        self.add_dynamic_obstacles = self.get_parameter('add_dynamic_obstacles').value
        
        # Publishers
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/current_pose',
            10
        )
        
        # Timer for publishing
        self.create_timer(1.0 / publish_frequency, self.publish_callback)
        
        # Initialize occupancy grid
        self.occupancy_grid = self.create_test_occupancy_grid()
        self.obstacle_position = 0
        
        self.get_logger().info('Mock planning node initialized')
        self.get_logger().info(f'Grid size: {self.grid_width}x{self.grid_height}')
        self.get_logger().info(f'Resolution: {self.grid_resolution} m/cell')
    
    def create_test_occupancy_grid(self) -> np.ndarray:
        """
        Create a test occupancy grid with obstacles.
        
        Returns:
            2D numpy array with occupancy values (0-100)
        """
        # Initialize empty grid
        grid = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        
        # Add border obstacles
        grid[0, :] = 100  # Top border
        grid[-1, :] = 100  # Bottom border
        grid[:, 0] = 100  # Left border
        grid[:, -1] = 100  # Right border
        
        # Add some static obstacles
        # Vertical wall
        grid[20:60, 40] = 100
        
        # Horizontal wall with gap
        grid[50, 10:35] = 100
        grid[50, 45:80] = 100
        
        # Scattered obstacles
        grid[30:35, 60:65] = 100
        grid[70:75, 30:35] = 100
        
        # L-shaped obstacle
        grid[60:70, 70:75] = 100
        grid[65:70, 70:80] = 100
        
        self.get_logger().info('Created test occupancy grid with static obstacles')
        
        return grid
    
    def add_moving_obstacle(self, grid: np.ndarray) -> np.ndarray:
        """
        Add a moving obstacle to simulate dynamic environment.
        
        Args:
            grid: Current occupancy grid
            
        Returns:
            Updated grid with moving obstacle
        """
        # Clear previous obstacle position
        grid[30:40, :] = np.minimum(grid[30:40, :], 50)  # Clear but keep static obstacles
        
        # Add obstacle at new position
        obstacle_col = int(self.obstacle_position % (self.grid_width - 20)) + 10
        grid[30:40, obstacle_col:obstacle_col+5] = 100
        
        # Update position for next iteration
        self.obstacle_position += 2
        
        return grid
    
    def publish_callback(self):
        """
        Callback to publish occupancy grid and current pose.
        """
        # Add dynamic obstacle if enabled
        if self.add_dynamic_obstacles:
            self.occupancy_grid = self.add_moving_obstacle(self.occupancy_grid.copy())
        
        # Create and publish occupancy grid message
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_height
        grid_msg.info.resolution = self.grid_resolution
        
        grid_msg.info.origin = Pose()
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = 0.0
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Flatten grid to 1D array
        grid_msg.data = self.occupancy_grid.flatten().tolist()
        
        self.occupancy_pub.publish(grid_msg)
        
        # Publish a mock current pose (simulating robot movement)
        pose_msg = PoseStamped()
        pose_msg.header = grid_msg.header
        
        # Simple linear motion along path
        time_sec = self.get_clock().now().seconds_nanoseconds()[0]
        progress = (time_sec % 20) / 20.0  # 20 second cycle
        
        pose_msg.pose.position.x = 0.5 + progress * 4.0
        pose_msg.pose.position.y = 0.5 + progress * 4.0
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockPlanningNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
