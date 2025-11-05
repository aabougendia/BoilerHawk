#!/usr/bin/env python3
"""Mock Planning Node for Testing Control."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math


class MockPlanningNode(Node):
    """Mock planning node that generates test trajectories."""

    def __init__(self):
        super().__init__('mock_planning_node')

        # Parameters
        self.declare_parameter('pattern', 'hover')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('altitude', 3.0)
        self.declare_parameter('size', 5.0)

        self.pattern = self.get_parameter('pattern').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.altitude = self.get_parameter('altitude').value
        self.size = self.get_parameter('size').value

        # State
        self.sequence = 0
        self.time = 0.0

        # Publisher
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/planning/setpoint', 10)

        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_setpoint)

        self.get_logger().info(
            f'Mock planning node initialized with pattern: {self.pattern}')

    def publish_setpoint(self):
        """Publish setpoint based on selected pattern."""

        setpoint = PoseStamped()
        setpoint.header.stamp = self.get_clock().now().to_msg()
        setpoint.header.frame_id = 'map'

        if self.pattern == 'hover':
            setpoint = self.generate_hover(setpoint)
        elif self.pattern == 'square':
            setpoint = self.generate_square(setpoint)
        elif self.pattern == 'circle':
            setpoint = self.generate_circle(setpoint)
        else:
            self.get_logger().warn(f'Unknown pattern: {self.pattern}')
            return

        self.setpoint_pub.publish(setpoint)
        self.time += 1.0 / self.publish_rate

    def generate_hover(self, setpoint):
        """Generate hover setpoint at fixed position."""
        setpoint.pose.position.x = 0.0
        setpoint.pose.position.y = 0.0
        setpoint.pose.position.z = self.altitude
        setpoint.pose.orientation.w = 1.0
        return setpoint

    def generate_square(self, setpoint):
        """Generate square pattern waypoints."""

        # Define square corners
        corners = [
            (self.size/2, self.size/2),
            (self.size/2, -self.size/2),
            (-self.size/2, -self.size/2),
            (-self.size/2, self.size/2),
        ]

        # Change waypoint every 5 seconds
        waypoint_index = int(self.time / 5.0) % 4

        setpoint.pose.position.x = corners[waypoint_index][0]
        setpoint.pose.position.y = corners[waypoint_index][1]
        setpoint.pose.position.z = self.altitude
        setpoint.pose.orientation.w = 1.0

        return setpoint

    def generate_circle(self, setpoint):
        """Generate circular trajectory."""

        # Circular motion with period of 20 seconds
        omega = 2.0 * math.pi / 20.0
        radius = self.size / 2.0

        setpoint.pose.position.x = radius * math.cos(omega * self.time)
        setpoint.pose.position.y = radius * math.sin(omega * self.time)
        setpoint.pose.position.z = self.altitude
        setpoint.pose.orientation.w = 1.0

        return setpoint


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MockPlanningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
