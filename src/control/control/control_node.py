#!/usr/bin/env python3
"""
Control Node for BoilerHawk.
Receives paths from planning module and sends setpoint commands to ArduPilot via MAVROS.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math


class ControlNode(Node):
    """
    ROS 2 node for drone control via MAVROS.
    Follows paths from planning module and sends position setpoints to ArduPilot.
    """
    
    def __init__(self):
        super().__init__('control_node')
        
        # Declare parameters
        self.declare_parameter('waypoint_threshold', 0.5)
        self.declare_parameter('setpoint_rate', 20.0)
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('auto_mode_switch', True)
        self.declare_parameter('target_altitude', 2.0)
        
        # Get parameters
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        setpoint_rate = self.get_parameter('setpoint_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_mode_switch = self.get_parameter('auto_mode_switch').value
        self.target_altitude = self.get_parameter('target_altitude').value
        
        # State variables
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.mavros_state = None
        self.target_setpoint = None
        
        # QoS profile for MAVROS compatibility
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            '/local_path',
            self.path_callback,
            10
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/control/status',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Timer for setpoint publishing (must be continuous for GUIDED mode)
        self.setpoint_timer = self.create_timer(
            1.0 / setpoint_rate,
            self.setpoint_callback
        )
        
        # Timer for status monitoring
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        self.get_logger().info('Control node initialized')
        self.get_logger().info(f'Waypoint threshold: {self.waypoint_threshold}m')
        self.get_logger().info(f'Setpoint rate: {setpoint_rate}Hz')
        self.get_logger().info(f'Auto arm: {self.auto_arm}')
        self.get_logger().info(f'Auto mode switch: {self.auto_mode_switch}')
    
    def path_callback(self, msg: Path):
        """
        Callback for receiving new paths from planning module.
        
        Args:
            msg: Path message containing waypoints
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        self.current_path = msg
        self.current_waypoint_idx = 0
        
        self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
        
        # Update target setpoint to first waypoint
        self.update_target_waypoint()
    
    def state_callback(self, msg: State):
        """
        Callback for MAVROS state updates.
        
        Args:
            msg: State message from MAVROS
        """
        prev_state = self.mavros_state
        self.mavros_state = msg
        
        # Log connection status changes
        if prev_state is None or prev_state.connected != msg.connected:
            if msg.connected:
                self.get_logger().info('Connected to flight controller')
            else:
                self.get_logger().warn('Disconnected from flight controller')
        
        # Log mode changes
        if prev_state is None or prev_state.mode != msg.mode:
            self.get_logger().info(f'Flight mode: {msg.mode}')
        
        # Handle auto mode switching
        if self.auto_mode_switch and msg.connected and msg.mode != 'GUIDED':
            self.request_guided_mode()
        
        # Handle auto arming
        if self.auto_arm and msg.connected and not msg.armed and msg.mode == 'GUIDED':
            self.request_arm()
    
    def pose_callback(self, msg: PoseStamped):
        """
        Callback for current pose updates from MAVROS.
        
        Args:
            msg: PoseStamped message with current drone position
        """
        self.current_pose = msg
        
        # Check if current waypoint is reached
        if self.current_path is not None and self.target_setpoint is not None:
            distance = self.calculate_distance(
                self.current_pose.pose.position,
                self.target_setpoint.pose.position
            )
            
            if distance < self.waypoint_threshold:
                self.advance_waypoint()
    
    def setpoint_callback(self):
        """
        Periodic callback to publish setpoint commands.
        This MUST run continuously for GUIDED mode to maintain control.
        """
        if self.target_setpoint is None:
            # If no waypoint available, hold current position
            if self.current_pose is not None:
                setpoint = PoseStamped()
                setpoint.header.stamp = self.get_clock().now().to_msg()
                setpoint.header.frame_id = 'map'
                setpoint.pose = self.current_pose.pose
                self.setpoint_pub.publish(setpoint)
            return
        
        # Update timestamp and publish target setpoint
        self.target_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target_setpoint)
    
    def status_callback(self):
        """
        Periodic callback to publish status information.
        """
        status_msg = String()
        
        if self.mavros_state is None:
            status_msg.data = 'Status: Waiting for MAVROS connection'
        elif not self.mavros_state.connected:
            status_msg.data = 'Status: Flight controller disconnected'
        elif self.current_path is None:
            status_msg.data = f'Status: No path - Mode: {self.mavros_state.mode} - Armed: {self.mavros_state.armed}'
        else:
            total_waypoints = len(self.current_path.poses)
            status_msg.data = (f'Status: Following path - Waypoint: {self.current_waypoint_idx + 1}/{total_waypoints} '
                             f'- Mode: {self.mavros_state.mode} - Armed: {self.mavros_state.armed}')
        
        self.status_pub.publish(status_msg)
    
    def update_target_waypoint(self):
        """
        Update the target setpoint to the current waypoint in the path.
        """
        if self.current_path is None or self.current_waypoint_idx >= len(self.current_path.poses):
            self.target_setpoint = None
            return
        
        waypoint = self.current_path.poses[self.current_waypoint_idx]
        
        # Create setpoint with target altitude
        self.target_setpoint = PoseStamped()
        self.target_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.target_setpoint.header.frame_id = 'map'
        self.target_setpoint.pose.position.x = waypoint.pose.position.x
        self.target_setpoint.pose.position.y = waypoint.pose.position.y
        self.target_setpoint.pose.position.z = self.target_altitude  # Use configured altitude
        self.target_setpoint.pose.orientation = waypoint.pose.orientation
        
        self.get_logger().info(
            f'Target waypoint {self.current_waypoint_idx + 1}: '
            f'({self.target_setpoint.pose.position.x:.2f}, '
            f'{self.target_setpoint.pose.position.y:.2f}, '
            f'{self.target_setpoint.pose.position.z:.2f})'
        )
    
    def advance_waypoint(self):
        """
        Advance to the next waypoint in the path.
        """
        if self.current_path is None:
            return
        
        self.current_waypoint_idx += 1
        
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.get_logger().info('Path complete! Reached final waypoint.')
            # Hold at final position
            return
        
        self.get_logger().info(f'Waypoint reached! Advancing to waypoint {self.current_waypoint_idx + 1}')
        self.update_target_waypoint()
    
    def calculate_distance(self, point1: Point, point2: Point) -> float:
        """
        Calculate Euclidean distance between two points.
        
        Args:
            point1: First point
            point2: Second point
            
        Returns:
            Distance in meters
        """
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        dz = point1.z - point2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def request_guided_mode(self):
        """
        Request the flight controller to switch to GUIDED mode.
        """
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Set mode service not available')
            return
        
        request = SetMode.Request()
        request.custom_mode = 'GUIDED'
        
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.set_mode_callback)
    
    def set_mode_callback(self, future):
        """
        Callback for set mode service response.
        """
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info('GUIDED mode request sent')
            else:
                self.get_logger().warn('Failed to set GUIDED mode')
        except Exception as e:
            self.get_logger().error(f'Set mode service call failed: {e}')
    
    def request_arm(self):
        """
        Request the flight controller to arm.
        """
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arming service not available')
            return
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.arm_callback)
    
    def arm_callback(self, future):
        """
        Callback for arming service response.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Arming successful')
            else:
                self.get_logger().warn('Arming failed')
        except Exception as e:
            self.get_logger().error(f'Arming service call failed: {e}')


def main(args=None):
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
