#!/usr/bin/env python3
"""
Launch file for the control node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for control node."""
    
    # Get package directory
    control_pkg_dir = get_package_share_directory('control')
    
    # Path to config file
    config_file = os.path.join(control_pkg_dir, 'config', 'control_params.yaml')
    
    # Control node
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        control_node,
    ])
