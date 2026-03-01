#!/usr/bin/env python3
"""
Full pipeline launch — single command starts EVERYTHING:

  ArduPilot SITL  →  Gazebo Harmonic  →  MAVROS  →  Perception  →  Planning  →  Control

No extra terminals needed.  Just run:
    ros2 launch sim_models stage123_control.launch.py

Composes:
  ArduPilot   sim_vehicle.py (SITL + MAVProxy with map & console)
  MAVROS      mavros_node   (fcu_url  udp://:14550@127.0.0.1:14555)
  Stage 1     Gazebo (outdoor_world_ardupilot) + ros_gz_bridge + TF + RViz
  Stage 2     perception_node    (/depth_camera/points → /occupancy_grid)
  Stage 3     planning_node      (/occupancy_grid → /global_path, /local_path)
  Stage 4     control_node       (/local_path → /mavros/setpoint_position/local)
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

    # ---- Launch Arguments (overridable from CLI) ----
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14550@127.0.0.1:14555',
        description='MAVLink FCU connection URL for MAVROS',
    )

    # Paths to model directories that Gazebo needs
    local_models_dir = os.path.join(pkg_sim_models, 'models')
    ardupilot_models_dir = os.path.expanduser('~/ardupilot_gazebo/models')
    ardupilot_plugin_dir = os.path.expanduser('~/ardupilot_gazebo/build')

    # Build GZ_SIM_RESOURCE_PATH: existing env + our models + ardupilot models
    existing_resource = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join(
        p for p in [existing_resource, local_models_dir, ardupilot_models_dir] if p
    )
    existing_plugin = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = os.pathsep.join(
        p for p in [existing_plugin, ardupilot_plugin_dir] if p
    )

    world_path = os.path.join(
        pkg_sim_models, 'worlds', 'outdoor_world_ardupilot.sdf'
    )
    rviz_config_path = os.path.join(pkg_sim_models, 'rviz', 'drone_outdoor.rviz')
    control_config = os.path.join(pkg_control, 'config', 'control_params.yaml')

    # MAVROS config files (shipped with the mavros package)
    mavros_pluginlists = os.path.join(pkg_mavros, 'launch', 'apm_pluginlists.yaml')
    mavros_config = os.path.join(pkg_mavros, 'launch', 'apm_config.yaml')

    # ============ Environment Variables for Gazebo ============

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=resource_path,
    )
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_path,
    )

    # ============ ArduPilot SITL + MAVProxy ============
    # Launches sim_vehicle.py with MAVProxy (--map --console) and forwards
    # MAVLink to MAVROS via --out udp:127.0.0.1:14550.
    # Delay 2 s to let Gazebo spin up first.

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
                ],
                output='log',
                shell=False,
            ),
        ],
    )

    # ============ MAVROS ============
    # Delay 25 s so SITL + MAVProxy are fully running before MAVROS connects.

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

    # ============ Stage 1: Gazebo + Bridges + TF ============

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r'],
        output='screen',
        shell=False,
    )

    # Depth camera bridges (iris_with_depth_cam publishes on "depth_camera" topic)
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

    # Static TF: world → odom
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )

    # Dynamic TF: odom → base_link (from MAVROS pose) + republish /current_pose
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

    # RViz (delayed to let bridges start)
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

    # ============ Stage 2: Perception ============

    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        parameters=[
            {'pointcloud_topic': '/depth_camera/points'},
            {'occupancy_topic': '/occupancy_grid'},
            {'pose_topic': '/mavros/local_position/pose'},
            {'resolution': 0.2},
            {'max_range': 5.0},
            {'inflate_radius': 0.6},
            {'grid_frame': 'map'},
        ],
        output='screen',
    )

    # ============ Stage 3: Planning ============

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

    # ============ Stage 4: Control (MAVROS interface) ============

    control_node = Node(
        package='control',
        executable='control_node',
        name='control_node',
        parameters=[control_config],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Launch arguments
        fcu_url_arg,
        # Environment variables for Gazebo model resolution
        set_resource_path,
        set_plugin_path,
        # ArduPilot SITL (delay 2 s)
        ardupilot_sitl,
        # MAVROS (delay 10 s — waits for SITL heartbeat)
        mavros_node,
        # Stage 1
        gazebo,
        bridge_camera_image,
        bridge_camera_depth,
        bridge_camera_points,
        bridge_camera_info,
        static_tf_world_odom,
        mavros_tf_broadcaster,
        rviz,
        # Stage 2
        perception_node,
        # Stage 3
        planning_node,
        # Stage 4
        control_node,
    ])
