#!/usr/bin/env python3
"""
Full pipeline launch: Stage 1 → Stage 2 → Stage 3 → Stage 4 (Control).

Uses the ArduPilot-compatible outdoor world with iris_with_depth_cam model.
MAVROS must be started externally — this launch does NOT start it.

Composes:
  Stage 1  Gazebo (outdoor_world_ardupilot) + ros_gz_bridge + TF + RViz
  Stage 2  perception_node    (/depth_camera/points → /occupancy_grid)
  Stage 3  planning_node      (/occupancy_grid → /global_path, /local_path)
  Stage 4  control_node       (/local_path → /mavros/setpoint_position/local)
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_sim_models = get_package_share_directory('sim_models')
    pkg_control = get_package_share_directory('control')

    world_path = os.path.join(
        pkg_sim_models, 'worlds', 'outdoor_world_ardupilot.sdf'
    )
    rviz_config_path = os.path.join(pkg_sim_models, 'rviz', 'drone_outdoor.rviz')
    control_config = os.path.join(pkg_control, 'config', 'control_params.yaml')

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
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen',
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
            {'resolution': 0.1},
            {'max_range': 5.0},
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
            {'lookahead_distance': 20},
            {'planning_frequency': 2.0},
            {'start_x': 0.0},
            {'start_y': 0.0},
            {'goal_x': 3.0},
            {'goal_y': 3.0},
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
