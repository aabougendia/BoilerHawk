"""Delivery Manager Node — bridges ROS 2 commands to Gazebo transport.

Copyright 2026 BoilerHawk — MIT License.

Subscribes to ``/delivery/command`` (std_msgs/String) and publishes
on the Gazebo transport topic ``/delivery/detach`` when a "detach"
command is received.  Uses ``ros_gz_bridge`` for the actual transport
bridging — this node converts the ROS-side command into the expected
message.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class DeliveryManagerNode(Node):
    """Bridge delivery commands to Gazebo detachable joint control."""

    def __init__(self) -> None:
        super().__init__("delivery_manager_node")

        # Subscribe to delivery commands from the strategy
        self.create_subscription(
            String, "/delivery/command", self._command_cb, 10
        )

        # Publish to the bridged Gazebo topic — DetachableJoint expects gz.msgs.Empty
        self._detach_pub = self.create_publisher(Empty, "/delivery/detach", 10)

        self._attached = True
        self.get_logger().info("Delivery Manager started — package attached")

    def _command_cb(self, msg: String) -> None:
        cmd = msg.data.strip().lower()

        if cmd == "detach" and self._attached:
            self._detach_pub.publish(Empty())
            self._attached = False
            self.get_logger().info("Package DETACHED")

        elif cmd == "attach" and not self._attached:
            # Attach is handled automatically by DetachableJoint when
            # models overlap; this just tracks state
            self._attached = True
            self.get_logger().info("Package ATTACHED (tracking only)")

        else:
            self.get_logger().debug(
                f"Ignored command '{cmd}' (attached={self._attached})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
