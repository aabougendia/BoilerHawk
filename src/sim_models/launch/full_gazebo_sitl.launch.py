#!/usr/bin/env python3
"""
Full Gazebo SITL launch file for BoilerHawk.

Launches:
  1. Gazebo Harmonic with outdoor_world.sdf (iris drone + depth camera)
  2. ros_gz_bridge for all sensors + /clock
  3. MAVROS (connects to ArduPilot SITL)
  4. Static TF transforms for sensor frames
  5. RViz2 for visualization

Prerequisites:
  - ArduPilot SITL must be started SEPARATELY before or after this launch:
      sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
  - Environment variables must be set (use run_gazebo_sitl.sh):
      GZ_SIM_SYSTEM_PLUGIN_PATH  -> ardupilot_gazebo plugin path
      GZ_SIM_RESOURCE_PATH       -> model search paths
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Package paths ---
    pkg_sim_models = get_package_share_directory('sim_models')

    world_path = os.path.expanduser(
        '~/BoilerHawk/src/sim_models/worlds/outdoor_world.sdf'
    )
    rviz_config_path = os.path.join(pkg_sim_models, 'rviz', 'drone_outdoor.rviz')
    mavros_pluginlists = os.path.join(pkg_sim_models, 'config', 'mavros_apm_pluginlists.yaml')

    # --- Launch arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2',
    )
    launch_mavros_arg = DeclareLaunchArgument(
        'mavros', default_value='true',
        description='Launch MAVROS node',
    )
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='tcp://127.0.0.1:5760',
        description='MAVROS FCU connection URL (default: SITL TCP port)',
    )
    fcu_protocol_arg = DeclareLaunchArgument(
        'fcu_protocol', default_value='v2.0',
        description='MAVLink protocol version',
    )

    # =====================================================================
    # 1. Gazebo Harmonic
    # =====================================================================
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r'],
        output='screen',
        shell=False,
    )

    # =====================================================================
    # 2. ROS-Gazebo Bridges
    # =====================================================================
    # Clock bridge – required for use_sim_time
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Depth camera bridges
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

    # IMU bridge
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
        output='screen',
    )

    # NavSat / GPS bridge  (model topic = "navsat")
    bridge_navsat = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
        ],
        output='screen',
    )

    # Magnetometer bridge
    bridge_mag = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer',
        ],
        output='screen',
    )

    # Air pressure bridge
    bridge_air_pressure = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure',
        ],
        output='screen',
    )

    # Pose bridge (model pose from Gazebo)
    bridge_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/iris_drone/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
        ],
        output='screen',
    )

    # =====================================================================
    # 3. MAVROS  (connects to ArduPilot SITL)
    # =====================================================================
    mavros_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('mavros')),
        actions=[
            TimerAction(
                period=8.0,   # give Gazebo + SITL time to start
                actions=[
                    Node(
                        package='mavros',
                        executable='mavros_node',
                        name='mavros',
                        output='screen',
                        respawn=True,
                        respawn_delay=5.0,
                        parameters=[
                            mavros_pluginlists,
                            {
                            'fcu_url': LaunchConfiguration('fcu_url'),
                            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                            'system_id': 1,
                            'component_id': 1,
                            'target_system_id': 1,
                            'target_component_id': 1,
                            },
                        ],
                    ),
                ],
            ),
        ],
    )

    # =====================================================================
    # 4. Static TF transforms
    # =====================================================================
    # world -> odom (identity)
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen',
    )

    # Camera link transforms (relative to base_link)
    static_tf_model_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.15', '0', '0.08', '0', '0.2', '0',
            'iris_drone', 'iris_drone/depth_camera_link',
        ],
        output='screen',
    )
    static_tf_camera_link_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.015', '0', '0', '0', '0', '0',
            'iris_drone/depth_camera_link',
            'iris_drone/depth_camera_link/depth_camera',
        ],
        output='screen',
    )

    # =====================================================================
    # 5. RViz2
    # =====================================================================
    rviz = GroupAction(
        condition=IfCondition(LaunchConfiguration('rviz')),
        actions=[
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        arguments=['-d', rviz_config_path],
                        parameters=[{
                            'use_sim_time': LaunchConfiguration('use_sim_time'),
                        }],
                        output='screen',
                    ),
                ],
            ),
        ],
    )

    # =====================================================================
    # Assemble launch description
    # =====================================================================
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        launch_rviz_arg,
        launch_mavros_arg,
        fcu_url_arg,
        fcu_protocol_arg,
        # Gazebo
        gazebo,
        # Bridges
        bridge_clock,
        bridge_camera_image,
        bridge_camera_depth,
        bridge_camera_points,
        bridge_camera_info,
        bridge_imu,
        bridge_navsat,
        bridge_mag,
        bridge_air_pressure,
        bridge_pose,
        # MAVROS
        mavros_node,
        # TF
        static_tf_world_odom,
        static_tf_model_camera_link,
        static_tf_camera_link_sensor,
        # RViz
        rviz,
    ])
