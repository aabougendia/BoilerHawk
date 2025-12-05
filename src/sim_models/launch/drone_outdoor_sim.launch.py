#!/usr/bin/env python3
"""
Launch file for Iris drone outdoor simulation with depth camera
Includes Gazebo, ROS 2 bridges for all sensors, RViz, and TF transforms
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_sim_models = get_package_share_directory('sim_models')
    
    # Expand paths
    world_path = os.path.expanduser('~/boilerHawk_ws/src/sim_models/worlds/outdoor_world.sdf')
    rviz_config_path = os.path.join(pkg_sim_models, 'rviz', 'drone_outdoor.rviz')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Gazebo simulation
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-r'],
        output='screen',
        shell=False
    )
    
    # ROS-Gazebo Bridge for depth camera RGB image
    bridge_camera_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for depth camera depth image
    bridge_camera_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for depth camera point cloud
    bridge_camera_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for camera info
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for IMU (from iris_with_standoffs)
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for GPS/NavSat
    bridge_gps = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/gps@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for magnetometer
    bridge_mag = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for air pressure
    bridge_air_pressure = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/air_pressure@sensor_msgs/msg/FluidPressure@gz.msgs.FluidPressure'
        ],
        output='screen'
    )
    
    # ROS-Gazebo Bridge for drone pose
    bridge_pose = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/iris_drone/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
        ],
        output='screen'
    )
    
    # Static transform: world -> odom
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        output='screen'
    )
    
    # Static transform: odom -> iris_drone (model frame, spawn position)
    static_tf_odom_model = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.5', '0', '0', '0', 'odom', 'iris_drone'],
        output='screen'
    )
    
    # Static transform: iris_drone -> iris_drone/depth_camera_link (camera attached to drone)
    static_tf_model_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.15', '0', '0.08', '0', '0.2', '0', 'iris_drone', 'iris_drone/depth_camera_link'],
        output='screen'
    )
    
    # Static transform: iris_drone/depth_camera_link -> iris_drone/depth_camera_link/depth_camera (sensor frame)
    static_tf_camera_link_sensor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.015', '0', '0', '0', '0', '0', 'iris_drone/depth_camera_link', 'iris_drone/depth_camera_link/depth_camera'],
        output='screen'
    )
    
    # RViz2 (delayed start to allow bridges to initialize)
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config_path],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        bridge_camera_image,
        bridge_camera_depth,
        bridge_camera_points,
        bridge_camera_info,
        bridge_imu,
        bridge_gps,
        bridge_mag,
        bridge_air_pressure,
        bridge_pose,
        static_tf_world_odom,
        static_tf_odom_model,
        static_tf_model_camera_link,
        static_tf_camera_link_sensor,
        rviz
    ])
