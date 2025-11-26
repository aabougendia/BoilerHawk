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
    perception_pkg_dir = get_package_share_directory('perception')
    control_pkg_dir = get_package_share_directory('control')
    
    # Path to config files
    perception_config = os.path.join(perception_pkg_dir, 'config', 'perception_params.yaml')
    control_config = os.path.join(control_pkg_dir, 'config', 'control_params.yaml')
    
    # Mock planning node (generates test occupancy grids)
    mock_planning_node = Node(
        package='perception',
        executable='mock_planning_node',
        name='mock_planning_node',
        output='screen',
        parameters=[{
            'grid_width': 100,
            'grid_height': 100,
            'grid_resolution': 0.1,
            'publish_frequency': 1.0,
            'add_dynamic_obstacles': False,
        }],
        emulate_tty=True,
    )
    
    # Perception node (path planning)
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[perception_config],
        emulate_tty=True,
    )
    
    # Control node (MAVROS interface)
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[control_config],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        mock_planning_node,
        perception_node,
        control_node,
    ])
