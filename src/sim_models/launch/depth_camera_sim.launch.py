from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get sim_models package share directory (portable, ROS2-compliant)
    pkg_share = get_package_share_directory('sim_models')

    world_path = os.path.join(pkg_share, 'worlds', 'main_world.sdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'depth_cam_config.rviz')

    return LaunchDescription([
        # 1️⃣ Start Gazebo with world
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # 2️⃣ Bridge Gazebo topics to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
            ],
            output='screen'
        ),

        # 3️⃣ Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'world',
                'depth_cam/camera_link/depth_camera_sensor'
            ],
            output='screen'
        ),

        # 4️⃣ RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        ),
    ])