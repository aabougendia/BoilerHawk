#!/usr/bin/env python3
"""
Full autonomous system launch file for BoilerHawk with ArduPilot SITL.
Launches the complete pipeline:
  MAVROS (ArduPilot ↔ ROS 2)
  + Gazebo Bridges (depth_camera topics)
  + Perception (PointCloud2 → OccupancyGrid)
  + Planning (OccupancyGrid → Path via A*)
  + Control (Path → MAVROS setpoints)

Gazebo and ArduPilot SITL must be started separately before this launch.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_planning = get_package_share_directory('planning')
    pkg_control = get_package_share_directory('control')

    # Config files
    planning_config = os.path.join(pkg_planning, 'config', 'planning_params.yaml')
    control_config = os.path.join(pkg_control, 'config', 'control_params.yaml')

    # ============ Launch Arguments ============
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time from Gazebo'
    )

    # ============ MAVROS (ArduPilot ↔ ROS 2) ============
    mavros_pluginlists = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm_pluginlists.yaml')
    mavros_config = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm_config.yaml')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        parameters=[
            mavros_pluginlists,
            mavros_config,
            {
                'fcu_url': 'tcp://127.0.0.1:5760',
                'gcs_url': '',
                'system_id': 1,
                'component_id': 240,
                'target_system_id': 1,
                'target_component_id': 1,
            },
        ],
        output='screen',
        emulate_tty=True,
    )

    # ============ Gazebo → ROS 2 Bridges ============
    # The iris_ardupilot_depth_cam model publishes depth camera on topic "depth_camera"
    bridge_camera_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth_points',
        arguments=[
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    bridge_camera_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth_image',
        arguments=[
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    bridge_camera_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_depth_depth',
        arguments=[
            '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    # ============ TF Transforms ============
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # ============ Perception Node ============
    # Subscribes: /depth_camera/points (PointCloud2)
    # Publishes:  /perception/occupancy (OccupancyGrid)
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        parameters=[{
            'pointcloud_topic': '/depth_camera/points',
            'occupancy_topic': '/perception/occupancy',
            'resolution': 0.2,
            'max_range': 10.0,
            'grid_frame': 'map',
        }],
        output='screen',
        emulate_tty=True,
    )

    # ============ Planning Node ============
    # Subscribes: /perception/occupancy (OccupancyGrid)
    # Publishes:  /local_path (Path), /global_path (Path)
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        parameters=[
            planning_config,
        ],
        output='screen',
        emulate_tty=True,
    )

    # ============ Control Node ============
    # Subscribes: /local_path (Path), /mavros/state, /mavros/local_position/pose
    # Publishes:  /mavros/setpoint_position/local (PoseStamped)
    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        parameters=[
            control_config,
            {'sitl_mode': True},
            {'auto_arm': True},
            {'auto_mode_switch': True},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        use_sim_time_arg,

        # MAVROS connection to ArduPilot SITL
        mavros_node,

        # Gazebo sensor bridges
        bridge_camera_points,
        bridge_camera_image,
        bridge_camera_depth,

        # TF
        static_tf_map_odom,

        # Autonomy pipeline: Perception → Planning → Control
        perception_node,
        planning_node,
        control_node,
    ])
