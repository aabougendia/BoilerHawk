#!/usr/bin/env python3
"""
Package Delivery simulation launch — single command starts EVERYTHING:

  ArduPilot SITL → Gazebo (delivery_world) → MAVROS → Perception → Planning
  → Control → Mission Manager → Delivery Manager

Usage:
    ros2 launch sim_models delivery_sim.launch.py
    ros2 launch sim_models delivery_sim.launch.py pickup_x:=2.0 pickup_y:=-1.0 dropoff_x:=20.0 dropoff_y:=18.0
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

    # Paths to model directories that Gazebo needs
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
        pkg_sim_models, 'worlds', 'delivery_world.sdf'
    )
    rviz_config_path = os.path.join(pkg_sim_models, 'rviz', 'drone_outdoor.rviz')
    control_config = os.path.join(pkg_control, 'config', 'control_params.yaml')
    mission_config = os.path.join(pkg_mission, 'config', 'mission_params.yaml')

    # MAVROS config files
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
                    '--map',
                    '--console',
                    '--out', 'udp:127.0.0.1:14550',
                    '-P', 'WPNAV_SPEED=1500',
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

    # ============ Gazebo (delivery_world) ============

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

    bridge_detach = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # DetachableJoint subscribes with gz.msgs.Empty (not Boolean)
            '/delivery/detach@std_msgs/msg/Empty@gz.msgs.Empty',
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

    # ============ Perception (adjusted grid for delivery world ~30×30m) ============

    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        parameters=[
            {'pointcloud_topic': '/depth_camera/points'},
            {'occupancy_topic': '/occupancy_grid'},
            {'pose_topic': '/mavros/local_position/pose'},
            {'resolution': 0.25},
            {'max_range': 8.0},
            {'inflate_radius': 0.4},
            {'grid_frame': 'map'},
            # Grid covers delivery world area (0,0) to (25,25) with margin
            {'grid_origin_x': -3.0},
            {'grid_origin_y': -5.0},
            {'grid_width_m': 30.0},
            {'grid_height_m': 32.0},
            # Ground-point filtering
            {'cam_pitch': 0.2},
            {'cam_z_offset': 0.05},
            {'min_obstacle_height': 0.5},
            # Hit-based detection
            {'min_hits': 5},
            {'hit_increment': 3},
            {'hit_decay': 1},
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
            {'lookahead_distance': 40},
            {'planning_frequency': 2.0},
            {'start_x': 0.0},
            {'start_y': 0.0},
            # Default goal — overridden by mission manager at runtime
            {'goal_x': 20.0},
            {'goal_y': 18.0},
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

    # ============ Mission Manager ============

    mission_manager_node = Node(
        package='mission_manager',
        executable='mission_manager_node',
        name='mission_manager_node',
        parameters=[
            mission_config,
            # Override defaults for package delivery
            {'strategy_name': 'package_delivery'},
            {'strategy_params':
                '{"pickup_x": 0.0, "pickup_y": 0.0, '
                '"dropoff_x": 20.0, "dropoff_y": 18.0, '
                '"altitude": 3.0, "dwell_time": 3.0}'},
            {'takeoff_altitude': 3.0},
            # Must match or exceed control_node waypoint_threshold (1.5m) so
            # the mission FSM transitions phases before the control node exhausts
            # its path and stops moving
            {'waypoint_threshold': 2.0},
            {'auto_start': True},
            {'auto_start_delay': 15.0},
        ],
        output='screen',
    )

    # ============ Delivery Manager ============

    delivery_manager_node = Node(
        package='mission_manager',
        executable='delivery_manager_node',
        name='delivery_manager_node',
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
        bridge_detach,
        # TF
        static_tf_world_odom,
        static_tf_world_map,
        mavros_tf_broadcaster,
        # Visualization
        rviz,
        # Pipeline
        perception_node,
        planning_node,
        control_node,
        # Application layer
        mission_manager_node,
        delivery_manager_node,
    ])
