#!/usr/bin/env python3
"""
Full system launch file for BoilerHawk.
Launches perception (with mock planning) and control nodes together.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for full system."""
    
    # Get package directories
    planning_pkg_dir = get_package_share_directory('planning')
    control_pkg_dir = get_package_share_directory('control')
    
    # Path to config files
    planning_config = os.path.join(planning_pkg_dir, 'config', 'planning_params.yaml')
    control_config = os.path.join(control_pkg_dir, 'config', 'control_params.yaml')
    
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
        }],
        # Mock publishes on /perception/occupancy and /mavlink/local_position/pose
        # (same topics as real perception + control)
        remappings=[
            ('/occupancy_grid', '/perception/occupancy'),
            ('/current_pose', '/mavlink/local_position/pose'),
        ],
        emulate_tty=True,
    )
    
    # Planning node (path planning)
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        output='screen',
        parameters=[planning_config],
        emulate_tty=True,
    )
    
    # Control node (PyMAVLink interface)
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        mock_perception_node,
        planning_node,
        control_node,
    ])
