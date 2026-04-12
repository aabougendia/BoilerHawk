#!/usr/bin/env python3
"""
Search & Rescue simulation launch — single command starts EVERYTHING:

  ArduPilot SITL → Gazebo (rescue_world) → MAVROS → Perception → Planning
  → Control → Mission Manager (search_and_rescue) → Human Detector

Usage:
    ros2 launch sim_models rescue_sim.launch.py
    ros2 launch sim_models rescue_sim.launch.py center_x:=10.0 center_y:=10.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim_models = get_package_share_directory('sim_models')
    pkg_control = get_package_share_directory('control')
    pkg_mavros = get_package_share_directory('mavros')
    pkg_mission = get_package_share_directory('mission_manager')

    # ---- Launch Arguments ----
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14550@127.0.0.1:14555',
        description='MAVLink FCU connection URL for MAVROS',
    )

    # Paths
    local_models_dir = os.path.join(pkg_sim_models, 'models')
    ardupilot_models_dir = os.path.expanduser('~/ardupilot_gazebo/models')
    ardupilot_plugin_dir = os.path.expanduser('~/ardupilot_gazebo/build')

    existing_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join(
        p for p in [existing_resource, local_models_dir, ardupilot_models_dir] if p
    )
    existing_plugin = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = os.pathsep.join(
        p for p in [existing_plugin, ardupilot_plugin_dir] if p
    )

    world_path = os.path.join(
        pkg_sim_models, 'worlds', 'rescue_world.sdf'
    )
    rviz_config_path = os.path.join(pkg_sim_models, 'rviz', 'drone_outdoor.rviz')
    control_config = os.path.join(pkg_control, 'config', 'control_params.yaml')
    mission_config = os.path.join(pkg_mission, 'config', 'mission_params.yaml')

    mavros_pluginlists = os.path.join(pkg_mavros, 'launch', 'apm_pluginlists.yaml')
    mavros_config = os.path.join(pkg_mavros, 'launch', 'apm_config.yaml')

    # ============ Environment Variables ============

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=resource_path,
    )
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_path,
    )

    # ============ ArduPilot SITL ============

    sim_vehicle_path = os.path.expanduser(
        '~/ardupilot/Tools/autotest/sim_vehicle.py'
    )
    ardupilot_sitl = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    sim_vehicle_path,
                    '-v', 'ArduCopter',
                    '-f', 'gazebo-iris',
                    '--model', 'JSON',
                    '--out', 'udp:127.0.0.1:14550',
                ],
                output='log',
                shell=False,
            ),
        ],
    )

    # ============ MAVROS (delay 25 s) ============

    mavros_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                namespace='mavros',
                output='log',
                parameters=[
                    mavros_pluginlists,
                    mavros_config,
                    {'fcu_url': LaunchConfiguration('fcu_url')},
                    {'gcs_url': ''},
                    {'tgt_system': 1},
                    {'tgt_component': 1},
                    {'fcu_protocol': 'v2.0'},
                ],
            ),
        ],
    )

    # ============ Gazebo (rescue_world) ============

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r'],
        output='screen',
        shell=False,
    )

    # ============ Sensor Bridges ============

    bridge_camera_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
    )

    bridge_camera_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        output='screen',
    )

    bridge_camera_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    # ============ TF ============

    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )

    static_tf_world_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
        output='screen',
    )

    mavros_tf_broadcaster = Node(
        package='sensors_interface',
        executable='mavros_tf_broadcaster',
        name='mavros_tf_broadcaster',
        parameters=[
            {'parent_frame': 'odom'},
            {'child_frame': 'base_link'},
            {'mavros_pose_topic': '/mavros/local_position/pose'},
            {'current_pose_topic': '/current_pose'},
        ],
        output='screen',
    )

    # ============ RViz ============

    rviz = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d', rviz_config_path,
                    '--ros-args', '--log-level', 'error',
                ],
                output='log',
            )
        ],
    )

    # ============ Perception (grid covers 20×20m search area) ============

    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        parameters=[
            {'pointcloud_topic': '/depth_camera/points'},
            {'occupancy_topic': '/occupancy_grid'},
            {'pose_topic': '/mavros/local_position/pose'},
            {'resolution': 0.25},
            {'max_range': 5.0},
            {'inflate_radius': 0.2},
            {'grid_frame': 'map'},
            {'grid_origin_x': -2.0},
            {'grid_origin_y': -2.0},
            {'grid_width_m': 25.0},
            {'grid_height_m': 25.0},
            {'cam_pitch': 0.2},
            {'cam_z_offset': 0.05},
            {'min_obstacle_height': 0.5},
            {'min_hits': 5},
            {'hit_increment': 2},
            {'hit_decay': 1},
        ],
        output='screen',
    )

    # ============ Human Detector ============

    human_detector_node = Node(
        package='perception',
        executable='human_detector_node',
        name='human_detector_node',
        parameters=[
            {'image_topic': '/depth_camera/image'},
            {'depth_topic': '/depth_camera/depth_image'},
            {'pose_topic': '/mavros/local_position/pose'},
            {'detection_topic': '/detection/pose'},
            {'detection_image_topic': '/detection/image'},
            {'detection_info_topic': '/detection/info'},
            {'confidence_threshold': 0.45},
            {'detect_rate': 5.0},
            {'consecutive_required': 3},
            {'cam_pitch': 0.2},
            {'cam_z_offset': 0.05},
            {'cam_hfov': 1.047},
        ],
        output='screen',
    )

    # ============ Planning ============

    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        parameters=[
            {'occupancy_threshold': 50},
            {'lookahead_distance': 30},
            {'planning_frequency': 2.0},
            {'start_x': 0.0},
            {'start_y': 0.0},
            {'goal_x': 10.0},
            {'goal_y': 10.0},
        ],
        output='screen',
    )

    # ============ Control ============

    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        parameters=[
            control_config,
            {'waypoint_threshold': 1.5},
            {'target_altitude': 3.0},
        ],
        output='screen',
        emulate_tty=True,
    )

    # ============ Mission Manager (search_and_rescue strategy) ============

    mission_manager_node = Node(
        package='mission_manager',
        executable='mission_manager_node',
        name='mission_manager_node',
        parameters=[
            mission_config,
            {'strategy_name': 'search_and_rescue'},
            {'strategy_params':
                '{"center_x": 15.0, "center_y": 12.0, '
                '"altitude": 3.0, "initial_leg": 2.0, '
                '"leg_increment": 2.0, "num_legs": 12, '
                '"hover_duration": 5.0, "max_detections": 1}'},
            {'takeoff_altitude': 3.0},
            {'waypoint_threshold': 2.0},
            {'auto_start': True},
            {'auto_start_delay': 15.0},
        ],
        output='screen',
    )

    return LaunchDescription([
        # Launch arguments
        fcu_url_arg,
        # Environment
        set_resource_path,
        set_plugin_path,
        # ArduPilot SITL (delay 2 s)
        ardupilot_sitl,
        # MAVROS (delay 25 s)
        mavros_node,
        # Gazebo
        gazebo,
        # Bridges
        bridge_camera_image,
        bridge_camera_depth,
        bridge_camera_points,
        bridge_camera_info,
        # TF
        static_tf_world_odom,
        static_tf_world_map,
        mavros_tf_broadcaster,
        # Visualization
        rviz,
        # Pipeline
        perception_node,
        human_detector_node,
        planning_node,
        control_node,
        # Application layer
        mission_manager_node,
    ])
