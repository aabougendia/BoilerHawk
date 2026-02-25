#!/usr/bin/env python3
"""
Full autonomous pipeline: perception + control (BendyRuler).
ArduPilot's built-in BendyRuler handles obstacle avoidance; planning node
is not needed in this pipeline.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch perception and control with BendyRuler obstacle avoidance."""
    control_pkg_dir = get_package_share_directory('control')
    control_config = os.path.join(control_pkg_dir, 'config', 'control_params.yaml')

    # Launch arguments
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/depth_camera/points',
        description='Point cloud topic from depth camera',
    )
    occupancy_topic_arg = DeclareLaunchArgument(
        'occupancy_topic',
        default_value='/perception/occupancy',
        description='Occupancy grid topic published by perception',
    )
    grid_frame_arg = DeclareLaunchArgument(
        'grid_frame',
        default_value='map',
        description='Frame ID for occupancy grid',
    )
    avoid_only_arg = DeclareLaunchArgument(
        'avoid_only',
        default_value='true',
        description='true = fly forward avoiding obstacles; false = navigate to goal',
    )

    # Perception: depth camera -> occupancy grid
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'occupancy_topic': LaunchConfiguration('occupancy_topic'),
            'resolution': 0.1,
            'max_range': 5.0,
            'grid_frame': LaunchConfiguration('grid_frame'),
        }],
        emulate_tty=True,
    )

    # Control: occupancy grid -> OBSTACLE_DISTANCE -> ArduPilot BendyRuler
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        output='screen',
        parameters=[
            control_config,
            {
                'occupancy_topic': LaunchConfiguration('occupancy_topic'),
                'avoid_only': LaunchConfiguration('avoid_only'),
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        pointcloud_topic_arg,
        occupancy_topic_arg,
        grid_frame_arg,
        avoid_only_arg,
        perception_node,
        control_node,
    ])
