#!/usr/bin/env python3
"""
Launch file for planning module with mock perception node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for planning testing."""
    
    # Mock perception node (generates test occupancy grids)
    mock_perception_node = Node(
        package='planning',
        executable='mock_perception_node',
        name='mock_perception_node',
        output='screen',
        parameters=[{
            'grid_width': 100,
            'grid_height': 100,
            'grid_resolution': 0.1,
            'publish_frequency': 1.0,
            'add_dynamic_obstacles': False,
        }]
    )
    
    # Planning node (processes occupancy grid and plans paths)
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{
            'occupancy_threshold': 50,
            'lookahead_distance': 20,
            'planning_frequency': 2.0,
            'start_x': 1.0,
            'start_y': 1.0,
            'goal_x': 8.0,
            'goal_y': 8.0,
        }]
    )
    
    return LaunchDescription([
        mock_perception_node,
        planning_node,
    ])
