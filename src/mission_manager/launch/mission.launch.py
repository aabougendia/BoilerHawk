"""Launch file for the mission manager node.

Copyright 2026 BoilerHawk — MIT License.

Usage:
    ros2 launch mission_manager mission.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the mission manager."""
    pkg_share = get_package_share_directory("mission_manager")

    config_file = os.path.join(pkg_share, "config", "mission_params.yaml")

    mission_manager_node = Node(
        package="mission_manager",
        executable="mission_manager_node",
        name="mission_manager_node",
        output="screen",
        parameters=[config_file],
    )

    return LaunchDescription([mission_manager_node])
