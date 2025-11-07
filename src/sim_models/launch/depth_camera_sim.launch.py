from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to your world
    world_path = os.path.expanduser(
        '~/boilerHawk_ws/src/sim_models/worlds/main_world.sdf'
    )

    # Path to your RViz configuration (optional)
    rviz_config_path = os.path.expanduser(
        '~/boilerHawk_ws/src/sim_models/rviz/depth_cam_config.rviz'
    )

    return LaunchDescription([
        # --- 1️⃣ Start Gazebo with your custom world ---
        ExecuteProcess(
            cmd=['gz', 'sim', world_path],
            output='screen'
        ),

        # --- 2️⃣ Bridge Gazebo topics to ROS 2 ---
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

        # --- 3️⃣ Static TF from world → camera_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_link'],
            output='screen'
        ),

        # --- 4️⃣ Optional: Launch RViz with preconfigured view ---
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            # condition=None  # remove if you don’t have the file yet
        )
    ])
