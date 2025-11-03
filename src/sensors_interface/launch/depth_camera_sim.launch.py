from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution
import os

def generate_launch_description():
    pkg_name = 'sensors_interface'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'depth_camera.xacro'])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])
    # rviz_file = PathJoinSubstitution([pkg_share, 'rviz', 'depth_camera.rviz'])

    # Instead of giving Gazebo a .xacro, we run xacro and give it the output
    urdf_processed = Command(['xacro ', xacro_file])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', '-r', world_file],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', urdf_processed,  # directly pass XML text to Gazebo
                '-name', 'depth_camera',
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            ],
            output='screen'
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_file],
        #     output='screen'
        # ),
    ])
