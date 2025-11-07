from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-file', '/home/aabougen/boilerHawk_ws/src/sim_models/models/depth_cam/model.sdf',
                       '-name', 'depth_cam'],
            output='screen'
        ),

        # Bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/points@sensor_msgs/msg/PointCloudPacked@gz.msgs.PointCloudPacked'
            ],
            output='screen'
        )
    ])
