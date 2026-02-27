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
from mavros_msgs.msg import State, OverrideRCIn
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
        self.declare_parameter('sitl_mode', True)  # Disable failsafes for SITL
        
        # Get parameters
        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        setpoint_rate = self.get_parameter('setpoint_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_mode_switch = self.get_parameter('auto_mode_switch').value
        self.target_altitude = self.get_parameter('target_altitude').value
        self.sitl_mode = self.get_parameter('sitl_mode').value
        
        # Failsafe override tracking
        self.failsafe_disabled = False
        
        # Takeoff state tracking
        self.takeoff_complete = False
        self.takeoff_requested = False
        
        # State variables
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None
        self.mavros_state = None
        self.target_setpoint = None
        
        # Diagnostic counters
        self.diag_path_received_count = 0
        self.diag_setpoint_published_count = 0
        self.diag_last_path_time = None
        self._last_path_fingerprint = None  # (count, first_xy, last_xy)
        self._path_complete_logged = False
        
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
        
        # RC Override publisher (for disabling throttle failsafe in SITL)
        self.rc_override_pub = self.create_publisher(
            OverrideRCIn,
            '/mavros/rc/override',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Import CommandTOL for takeoff service
        from mavros_msgs.srv import CommandTOL
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Timer for setpoint publishing (must be continuous for GUIDED mode)
        self.setpoint_timer = self.create_timer(
            1.0 / setpoint_rate,
            self.setpoint_callback
        )
        
        # Timer for status monitoring
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        # Timer for detailed diagnostics (every 30 seconds)
        self.diag_timer = self.create_timer(30.0, self.diagnostics_callback)
        
        self.get_logger().info('Control node initialized')
        self.get_logger().info(f'Waypoint threshold: {self.waypoint_threshold}m')
        self.get_logger().info(f'Setpoint rate: {setpoint_rate}Hz')
        self.get_logger().info(f'Auto arm: {self.auto_arm}')
        self.get_logger().info(f'Auto mode switch: {self.auto_mode_switch}')
        self.get_logger().info(f'SITL mode (failsafes disabled): {self.sitl_mode}')
    
    def disable_throttle_failsafe(self):
        """
        Disable throttle failsafe by setting RC channel 3 (throttle) to mid position.
        This prevents ArduPilot from triggering failsafe due to low throttle in SITL.
        """
        if not self.sitl_mode:
            return
        
        rc_msg = OverrideRCIn()
        # Set all channels to 0 (no override) except channel 3 (throttle)
        rc_msg.channels = [0, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.rc_override_pub.publish(rc_msg)
        
        if not self.failsafe_disabled:
            self.get_logger().info('Throttle failsafe disabled (RC3=1500)')
            self.failsafe_disabled = True
    
    def path_callback(self, msg: Path):
        """
        Callback for receiving new paths from planning module.
        
        Args:
            msg: Path message containing waypoints
        """
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        
        # Build a fingerprint to detect genuinely new paths
        first = msg.poses[0].pose.position
        last = msg.poses[-1].pose.position
        fp = (len(msg.poses),
              round(first.x, 2), round(first.y, 2),
              round(last.x, 2), round(last.y, 2))
        path_changed = (fp != self._last_path_fingerprint)
        self._last_path_fingerprint = fp
        
        self.current_path = msg
        self.diag_path_received_count += 1
        self.diag_last_path_time = self.get_clock().now()
        
        # Find the closest waypoint, then skip forward past all waypoints
        # that are already within the reached threshold so the drone never
        # targets an already-passed waypoint.
        best_idx = 0
        if self.current_pose is not None:
            cx = self.current_pose.pose.position.x
            cy = self.current_pose.pose.position.y
            # 1. Find closest waypoint by 2-D distance
            min_dist = float('inf')
            for i, pose_st in enumerate(msg.poses):
                wp = pose_st.pose.position
                d = math.sqrt((wp.x - cx)**2 + (wp.y - cy)**2)
                if d < min_dist:
                    min_dist = d
                    best_idx = i
            # 2. Advance past waypoints already within the reached threshold
            while best_idx < len(msg.poses) - 1:
                wp = msg.poses[best_idx].pose.position
                d = math.sqrt((wp.x - cx)**2 + (wp.y - cy)**2)
                if d < self.waypoint_threshold:
                    best_idx += 1
                else:
                    break
        self.current_waypoint_idx = best_idx
        
        if path_changed:
            self._path_complete_logged = False
            self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints')
        
        # Update target setpoint to first waypoint
        self.update_target_waypoint(log=path_changed)
    
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
                # Disable throttle failsafe when connected (for SITL)
                self.disable_throttle_failsafe()
            else:
                self.get_logger().warn('Disconnected from flight controller')
        
        # Request takeoff when armed and in GUIDED mode (only if auto_arm is on)
        if self.auto_arm and msg.armed and msg.mode == 'GUIDED' and not self.takeoff_requested:
            self.get_logger().info('Armed in GUIDED mode - requesting takeoff')
            if self.request_takeoff():
                self.takeoff_requested = True
        
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
        
        # Robust takeoff detection: If we are at target altitude, assume takeoff complete
        # This handles cases where the service callback might have been missed
        if not self.takeoff_complete and \
           self.mavros_state is not None and \
           self.mavros_state.armed and \
           self.mavros_state.mode == 'GUIDED' and \
           msg.pose.position.z >= (self.target_altitude * 0.9):
            self.get_logger().info(f'Altitude {msg.pose.position.z:.2f}m reached - marking takeoff complete')
            self.takeoff_complete = True
            
        # Check if current waypoint is reached
        if self.current_path is not None and self.target_setpoint is not None:
            distance = self.calculate_distance(
                self.current_pose.pose.position,
                self.target_setpoint.pose.position
            )
            
            # Debug logging (throttled to 5 s)
            now_ns = self.get_clock().now().nanoseconds
            if not hasattr(self, '_last_dist_log') or (now_ns - self._last_dist_log) > 5_000_000_000:
                self.get_logger().info(
                    f'Dist: {distance:.2f}m | '
                    f'Pos: ({self.current_pose.pose.position.x:.2f}, {self.current_pose.pose.position.y:.2f}) | '
                    f'Tgt: ({self.target_setpoint.pose.position.x:.2f}, {self.target_setpoint.pose.position.y:.2f})'
                )
                self._last_dist_log = now_ns
            
            if distance < self.waypoint_threshold:
                self.advance_waypoint()
    
    def setpoint_callback(self):
        """
        Periodic callback to publish setpoint commands.
        This MUST run continuously for GUIDED mode to maintain control.
        """
        # Keep RC override active to prevent failsafe
        if self.sitl_mode and self.mavros_state is not None and self.mavros_state.connected:
            self.disable_throttle_failsafe()
        
        # Wait for takeoff to complete before sending setpoints
        if not self.takeoff_complete:
            # Log why we're not publishing (throttle to 1Hz)
            now = self.get_clock().now().nanoseconds
            if not hasattr(self, '_last_wait_log') or (now - self._last_wait_log) > 1e9:
                self.get_logger().warn(
                    f'[DIAG] Waiting for takeoff - takeoff_complete={self.takeoff_complete}, '
                    f'takeoff_requested={self.takeoff_requested}'
                )
                self._last_wait_log = now
            return

        if self.target_setpoint is None:
            # If no waypoint available, hold current position
            if self.current_pose is not None:
                setpoint = PoseStamped()
                setpoint.header.stamp = self.get_clock().now().to_msg()
                setpoint.header.frame_id = 'map'
                setpoint.pose = self.current_pose.pose
                self.setpoint_pub.publish(setpoint)
                self.diag_setpoint_published_count += 1
            return
        
        # Update timestamp and publish target setpoint
        self.target_setpoint.header.stamp = self.get_clock().now().to_msg()
        self.setpoint_pub.publish(self.target_setpoint)
        self.diag_setpoint_published_count += 1
        
        # Debug log (throttle to 1Hz)
        now = self.get_clock().now().nanoseconds
        if not hasattr(self, '_last_setpoint_log') or (now - self._last_setpoint_log) > 1e9:
            self.get_logger().info(
                f'Publishing setpoint: ({self.target_setpoint.pose.position.x:.2f}, '
                f'{self.target_setpoint.pose.position.y:.2f}, '
                f'{self.target_setpoint.pose.position.z:.2f})'
            )
            self._last_setpoint_log = now
    
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
    
    def diagnostics_callback(self):
        """
        Periodic callback to publish detailed diagnostics for debugging.
        Runs every 5 seconds with comprehensive system state.
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('[DIAGNOSTICS] Control Node State Summary')
        self.get_logger().info('=' * 60)
        
        # MAVROS connection state
        if self.mavros_state is None:
            self.get_logger().warn('[DIAG] MAVROS: NOT CONNECTED (state is None)')
        else:
            self.get_logger().info(
                f'[DIAG] MAVROS: connected={self.mavros_state.connected}, '
                f'mode={self.mavros_state.mode}, armed={self.mavros_state.armed}'
            )
        
        # Takeoff state
        self.get_logger().info(
            f'[DIAG] TAKEOFF: requested={self.takeoff_requested}, '
            f'complete={self.takeoff_complete}'
        )
        
        # Current position
        if self.current_pose is None:
            self.get_logger().warn('[DIAG] POSITION: No pose received yet')
        else:
            pos = self.current_pose.pose.position
            self.get_logger().info(
                f'[DIAG] POSITION: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}'
            )
        
        # Path state
        if self.current_path is None:
            self.get_logger().warn('[DIAG] PATH: No path received!')
        else:
            self.get_logger().info(
                f'[DIAG] PATH: {len(self.current_path.poses)} waypoints, '
                f'current_idx={self.current_waypoint_idx}'
            )
        
        # Target setpoint
        if self.target_setpoint is None:
            self.get_logger().warn('[DIAG] TARGET: No target setpoint set')
        else:
            tgt = self.target_setpoint.pose.position
            self.get_logger().info(
                f'[DIAG] TARGET: x={tgt.x:.2f}, y={tgt.y:.2f}, z={tgt.z:.2f}'
            )
        
        # Counters
        self.get_logger().info(
            f'[DIAG] COUNTERS: paths_received={self.diag_path_received_count}, '
            f'setpoints_published={self.diag_setpoint_published_count}'
        )
        
        # Time since last path
        if self.diag_last_path_time is not None:
            elapsed = (self.get_clock().now() - self.diag_last_path_time).nanoseconds / 1e9
            self.get_logger().info(f'[DIAG] Last path received {elapsed:.1f}s ago')
        
        # Check for common issues
        self.get_logger().info('-' * 40)
        self.get_logger().info('[DIAG] Issue Detection:')
        
        issues_found = False
        
        if self.mavros_state is None or not self.mavros_state.connected:
            self.get_logger().error('[ISSUE] MAVROS not connected!')
            issues_found = True
        
        if self.mavros_state and self.mavros_state.mode != 'GUIDED':
            self.get_logger().error(
                f'[ISSUE] Not in GUIDED mode (current: {self.mavros_state.mode})'
            )
            issues_found = True
        
        if self.mavros_state and not self.mavros_state.armed:
            self.get_logger().warn('[ISSUE] Drone not armed')
            issues_found = True
        
        if not self.takeoff_complete:
            self.get_logger().warn(
                '[ISSUE] Takeoff not complete - setpoints NOT being published!'
            )
            issues_found = True
        
        if self.current_path is None:
            self.get_logger().error(
                '[ISSUE] No path received from /local_path topic!'
            )
            issues_found = True
        
        if self.diag_setpoint_published_count == 0 and self.takeoff_complete:
            self.get_logger().error(
                '[ISSUE] Takeoff complete but no setpoints published!'
            )
            issues_found = True
        
        if not issues_found:
            self.get_logger().info('[DIAG] No issues detected - system OK')
        
        self.get_logger().info('=' * 60)
    
    def update_target_waypoint(self, log=True):
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
        
        if log:
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
            if not self._path_complete_logged:
                self.get_logger().info('Path complete! Reached final waypoint.')
                self._path_complete_logged = True
            # Hold at final position
            return
        
        self.get_logger().info(f'Waypoint reached! Advancing to waypoint {self.current_waypoint_idx + 1}')
        self.update_target_waypoint(log=True)
    
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
    
    def request_takeoff(self):
        """
        Request takeoff to target altitude.
        
        Returns:
            bool: True if request sent successfully
        """
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Takeoff service not available')
            return False
        
        from mavros_msgs.srv import CommandTOL
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = float(self.target_altitude)
        
        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self.takeoff_callback)
        self.get_logger().info(f'Takeoff requested to {self.target_altitude}m')
        return True
    
    def takeoff_callback(self, future):
        """
        Callback for takeoff service response.
        """
        try:
            from mavros_msgs.srv import CommandTOL
            response = future.result()
            if response.success:
                self.get_logger().info('Takeoff command accepted')
                # Give it time to climb before declaring complete
                self.takeoff_timer = self.create_timer(3.0, self.mark_takeoff_complete, one_shot=True)
            else:
                self.get_logger().warn('Takeoff command failed')
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {e}')
    
    def mark_takeoff_complete(self):
        """Mark takeoff as complete after delay."""
        self.takeoff_complete = True
        self.get_logger().info('Takeoff complete - waypoint following active')


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
