#!/usr/bin/env python3
"""Control Node for Autonomous Drone with MAVROS integration."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.qos import DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import String

import math
from enum import IntEnum


class ControlState(IntEnum):
    """Control node states."""
    IDLE = 0
    ARMING = 1
    TAKING_OFF = 2
    EXECUTING = 3
    HOVERING = 4
    LANDING = 5
    EMERGENCY = 6


class ControlNode(Node):
    """Control node for autonomous drone."""

    def __init__(self):
        super().__init__('control_node')

        # Parameters
        self.declare_parameter('update_rate', 20.0)
        self.declare_parameter('setpoint_timeout', 1.0)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('position_tolerance', 0.5)
        self.declare_parameter('takeoff_altitude', 2.0)

        self.update_rate = self.get_parameter('update_rate').value
        self.setpoint_timeout = (
            self.get_parameter('setpoint_timeout').value
        )
        self.max_velocity = self.get_parameter('max_velocity').value
        self.position_tolerance = (
            self.get_parameter('position_tolerance').value
        )
        self.takeoff_altitude = (
            self.get_parameter('takeoff_altitude').value
        )

        # State variables
        self.state = ControlState.IDLE
        self.mavros_state = State()
        self.current_pose = PoseStamped()
        self.current_velocity = TwistStamped()
        self.current_setpoint = PoseStamped()
        self.last_setpoint_time = self.get_clock().now()
        self.battery_status = BatteryState()
        self.gps_status = NavSatFix()
        self.takeoff_position = None
        self.home_position = None

        # QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, qos_profile
        )
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self.velocity_callback, qos_profile
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery',
            self.battery_callback, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.gps_callback, qos_profile
        )
        self.setpoint_sub = self.create_subscription(
            PoseStamped, '/planning/setpoint',
            self.setpoint_callback, 10
        )

        # Publishers
        self.setpoint_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        self.setpoint_vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10
        )
        self.status_pub = self.create_publisher(
            String, '/control/status', 10
        )

        # Service clients
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming'
        )
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode'
        )

        # Control loop timer
        self.timer = self.create_timer(
            1.0 / self.update_rate, self.control_loop
        )

        self.get_logger().info('Control node initialized')

    def state_callback(self, msg):
        """MAVROS state callback."""
        self.mavros_state = msg

    def pose_callback(self, msg):
        """Local position callback."""
        self.current_pose = msg
        if self.home_position is None:
            self.home_position = msg.pose.position

    def velocity_callback(self, msg):
        """Velocity callback."""
        self.current_velocity = msg

    def battery_callback(self, msg):
        """Battery status callback."""
        self.battery_status = msg

    def gps_callback(self, msg):
        """GPS status callback."""
        self.gps_status = msg

    def setpoint_callback(self, msg):
        """Planning setpoint callback."""
        if not self.mavros_state.connected:
            return
        self.current_setpoint = msg
        self.last_setpoint_time = self.get_clock().now()
        if self.state == ControlState.IDLE:
            self.state = ControlState.EXECUTING

    def control_loop(self):
        """Main control loop."""
        if not self.mavros_state.connected:
            if self.state != ControlState.IDLE:
                self.state = ControlState.IDLE
            self.publish_status()
            return

        setpoint_age = (
            (self.get_clock().now() - self.last_setpoint_time)
            .nanoseconds / 1e9
        )
        if (setpoint_age > self.setpoint_timeout and
                self.state == ControlState.EXECUTING):
            self.state = ControlState.HOVERING

        if self.state == ControlState.IDLE:
            pass
        elif self.state == ControlState.EXECUTING:
            target_pose = PoseStamped()
            target_pose.header.stamp = (
                self.get_clock().now().to_msg()
            )
            target_pose.header.frame_id = 'map'
            target_pose.pose = self.current_setpoint.pose
            self.setpoint_pos_pub.publish(target_pose)
        elif self.state == ControlState.HOVERING:
            target_pose = PoseStamped()
            target_pose.header.stamp = (
                self.get_clock().now().to_msg()
            )
            target_pose.header.frame_id = 'map'
            target_pose.pose = self.current_pose.pose
            self.setpoint_pos_pub.publish(target_pose)

        self.publish_status()

    def publish_status(self):
        """Publish control status."""
        status_msg = String()
        status_msg.data = (
            f'State: {self.state.name} | '
            f'Connected: {self.mavros_state.connected} | '
            f'Armed: {self.mavros_state.armed}'
        )
        self.status_pub.publish(status_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
