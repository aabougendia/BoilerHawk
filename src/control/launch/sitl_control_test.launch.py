#!/usr/bin/env python3
"""
Launch file for SITL testing with MAVROS and control node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for SITL control testing"""
    
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    fcu_url = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14550@127.0.0.1:14555',
        description='MAVROS FCU URL')
    
    pattern = DeclareLaunchArgument(
        'pattern',
        default_value='hover',
        description='Mock planning pattern: hover, square, circle')

    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'system_id': 1,
            'component_id': 240,
            'fcu_protocol': 'v2.0',
        }],
        respawn=True,
        respawn_delay=2.0
    )

    # Control node
    control_node = Node(
        package='control',
        executable='control_node.py',
        name='control',
        output='screen',
        parameters=[{
            'update_rate': 20.0,
            'setpoint_timeout': 1.0,
            'max_velocity': 5.0,
            'max_acceleration': 2.0,
            'position_tolerance': 0.5,
            'takeoff_altitude': 3.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # Mock planning node
    mock_planning_node = Node(
        package='control',
        executable='mock_planning_node.py',
        name='mock_planning',
        output='screen',
        parameters=[{
            'pattern': LaunchConfiguration('pattern'),
            'publish_rate': 10.0,
            'altitude': 3.0,
            'size': 5.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time,
        fcu_url,
        pattern,
        mavros_node,
        control_node,
        mock_planning_node,
    ])
