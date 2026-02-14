#!/usr/bin/env python3
"""
Dynamic TF Broadcaster + Pose Republisher.

Subscribes to /mavros/local_position/pose and:
  1. Broadcasts odom → base_link transform (dynamic TF)
  2. Republishes on /current_pose for the planning node
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class MavrosTfBroadcaster(Node):
    """Bridges MAVROS local pose into TF tree and /current_pose topic."""

    def __init__(self):
        super().__init__('mavros_tf_broadcaster')

        # Parameters
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('mavros_pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('current_pose_topic', '/current_pose')

        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value
        mavros_topic = self.get_parameter('mavros_pose_topic').value
        pose_topic = self.get_parameter('current_pose_topic').value

        self.parent_frame = parent_frame
        self.child_frame = child_frame

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher: /current_pose
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        # Subscriber: MAVROS local position (BEST_EFFORT to match MAVROS QoS)
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseStamped,
            mavros_topic,
            self._pose_cb,
            mavros_qos,
        )

        self.get_logger().info(
            f'TF broadcaster started: {mavros_topic} → '
            f'TF({parent_frame} → {child_frame}) + {pose_topic}'
        )

    def _pose_cb(self, msg: PoseStamped):
        """Forward MAVROS pose as dynamic TF and /current_pose."""
        # --- Publish TF ---
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # --- Republish as /current_pose ---
        self.pose_pub.publish(msg)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = MavrosTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
