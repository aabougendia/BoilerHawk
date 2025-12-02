#!/usr/bin/env python3
"""
Launch file for planning module with dynamic obstacles.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for planning testing with dynamic obstacles."""
    
    # Mock perception node with dynamic obstacles
    mock_perception_node = Node(
        package='planning',
        executable='mock_perception_node',
        name='mock_perception_node',
        output='screen',
        parameters=[{
            'grid_width': 100,
            'grid_height': 100,
            'grid_resolution': 0.1,
            'publish_frequency': 2.0,
            'add_dynamic_obstacles': True,
        }]
    )
    
    # Planning node
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[{
            'occupancy_threshold': 50,
            'lookahead_distance': 25,
            'planning_frequency': 5.0,
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
