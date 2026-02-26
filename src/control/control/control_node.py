#!/usr/bin/env python3
"""
Control Node for BoilerHawk (PyMAVLink version).
Communicates directly with ArduPilot via MAVLink over TCP.
Receives paths from planning module and sends position commands.

State machine:
  IDLE → SETTING_GUIDED → ARMING → TAKING_OFF → HOVERING → FLYING → LANDING → LANDED
"""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from pymavlink import mavutil


from control.flight_state import FlightState


class ControlNode(Node):
    """
    ROS 2 node for drone control via direct MAVLink (pymavlink).
    """

    # ArduPilot mode numbers
    GUIDED_MODE = 4
    LAND_MODE = 9
    MODE_MAP = {
        0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
        5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD',
    }

    def __init__(self):
        super().__init__('control_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('waypoint_threshold', 0.5)
        self.declare_parameter('setpoint_rate', 20.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_mode_switch', True)
        self.declare_parameter('target_altitude', 2.0)
        self.declare_parameter('fcu_connection', 'tcp:127.0.0.1:5760')
        self.declare_parameter('mavlink_poll_rate', 50.0)
        self.declare_parameter('auto_land_on_complete', False)
        self.declare_parameter('heartbeat_timeout', 5.0)

        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        setpoint_rate = self.get_parameter('setpoint_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_mode_switch = self.get_parameter('auto_mode_switch').value
        self.target_altitude = self.get_parameter('target_altitude').value
        fcu_connection = self.get_parameter('fcu_connection').value
        mavlink_poll_rate = self.get_parameter('mavlink_poll_rate').value
        self.auto_land_on_complete = self.get_parameter('auto_land_on_complete').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value

        # ── State variables ──────────────────────────────────────────
        self.flight_state = FlightState.IDLE
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None          # PoseStamped (NED → ENU converted)
        self.target_setpoint = None       # PoseStamped

        self.connected = False
        self.armed = False
        self.mode = 'UNKNOWN'
        self.last_heartbeat_time = None
        self.emergency_hold_active = False

        # Counters
        self.diag_path_count = 0
        self.diag_setpoint_count = 0
        self.diag_heartbeat_count = 0

        # ── MAVLink connection ───────────────────────────────────────
        self.get_logger().info(f'Connecting to FCU: {fcu_connection}')
        self.mav_conn = mavutil.mavlink_connection(fcu_connection)
        self.get_logger().info('Waiting for heartbeat...')
        self.mav_conn.wait_heartbeat(timeout=30)
        self.connected = True
        self.last_heartbeat_time = time.monotonic()
        self.get_logger().info(
            f'Heartbeat received (system {self.mav_conn.target_system} '
            f'comp {self.mav_conn.target_component})'
        )

        # Request data streams from ArduPilot (position, attitude, etc.)
        self._request_data_streams()

        # ── ROS 2 Subscribers ────────────────────────────────────────
        self.path_sub = self.create_subscription(
            Path, '/local_path', self.path_callback, 10)

        # Command subscriber for external control (takeoff, land, hold)
        self.command_sub = self.create_subscription(
            String, '/control/command', self.command_callback, 10)

        # Danger subscriber from planning (emergency hold)
        self.danger_sub = self.create_subscription(
            String, '/planning/danger', self.danger_callback, 10)

        # ── ROS 2 Publishers ─────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/control/status', 10)
        # Publish pose so RViz / other nodes can use it
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavlink/local_position/pose', 10)

        # ── Timers ───────────────────────────────────────────────────
        self.create_timer(1.0 / mavlink_poll_rate, self._mavlink_loop)
        self.create_timer(1.0 / setpoint_rate, self._setpoint_loop)
        self.create_timer(1.0, self._status_loop)
        self.create_timer(5.0, self._diagnostics_loop)
        self.create_timer(2.0, self._state_machine_loop)

        self.get_logger().info('Control node initialized (PyMAVLink)')
        self.get_logger().info(f'  waypoint_threshold={self.waypoint_threshold}m')
        self.get_logger().info(f'  setpoint_rate={setpoint_rate}Hz')
        self.get_logger().info(f'  auto_arm={self.auto_arm}')
        self.get_logger().info(f'  target_altitude={self.target_altitude}m')
        self.get_logger().info(f'  auto_land_on_complete={self.auto_land_on_complete}')

    # =================================================================
    #  State Machine
    # =================================================================

    def _transition_to(self, new_state: str) -> bool:
        """
        Attempt a state transition. Returns True if valid, False if rejected.
        """
        valid = FlightState.TRANSITIONS.get(self.flight_state, set())
        if new_state not in valid:
            self.get_logger().warn(
                f'Invalid transition: {self.flight_state} → {new_state}')
            return False
        old = self.flight_state
        self.flight_state = new_state
        self.get_logger().info(f'State: {old} → {new_state}')
        return True

    def _state_machine_loop(self):
        """
        Runs every 2s. Drives the automatic state progression.
        """
        if not self.connected:
            return

        state = self.flight_state

        if state == FlightState.IDLE:
            if self.auto_mode_switch:
                self._transition_to(FlightState.SETTING_GUIDED)

        elif state == FlightState.SETTING_GUIDED:
            if self.mode == 'GUIDED':
                if self.auto_arm:
                    self._transition_to(FlightState.ARMING)
            else:
                self.get_logger().info(
                    f'Current mode: {self.mode} → requesting GUIDED')
                self._send_set_mode(self.GUIDED_MODE)

        elif state == FlightState.ARMING:
            if self.armed:
                self._transition_to(FlightState.TAKING_OFF)
            else:
                self.get_logger().info('Requesting ARM')
                self._send_arm()

        elif state == FlightState.TAKING_OFF:
            if self.armed and self.mode == 'GUIDED':
                self.get_logger().info('Requesting TAKEOFF')
                self._send_takeoff(self.target_altitude)
                # Transition to HOVERING is handled in _mavlink_loop
                # when altitude threshold is reached

        elif state == FlightState.HOVERING:
            # If we have a path, start flying
            if self.current_path is not None and self.target_setpoint is not None:
                self._transition_to(FlightState.FLYING)

        elif state == FlightState.FLYING:
            # Flying is handled by _setpoint_loop and _mavlink_loop
            pass

        elif state == FlightState.LANDING:
            if not self.armed:
                self._transition_to(FlightState.LANDED)

        elif state == FlightState.LANDED:
            # Stay landed unless externally commanded
            pass

    # =================================================================
    #  Command Handling
    # =================================================================

    def command_callback(self, msg: String):
        """Handle external commands: takeoff, land, hold."""
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received command: {cmd}')

        if cmd == 'takeoff':
            if self.flight_state == FlightState.LANDED:
                self._transition_to(FlightState.SETTING_GUIDED)
            elif self.flight_state == FlightState.IDLE:
                self._transition_to(FlightState.SETTING_GUIDED)

        elif cmd == 'land':
            if self.flight_state in (FlightState.HOVERING, FlightState.FLYING,
                                     FlightState.EMERGENCY_HOLD):
                self._send_land()
                self._transition_to(FlightState.LANDING)

        elif cmd == 'hold':
            if self.flight_state == FlightState.FLYING:
                self.target_setpoint = None
                self._transition_to(FlightState.HOVERING)

    def danger_callback(self, msg: String):
        """Handle danger alerts from planning node."""
        danger = msg.data.strip().upper()

        if danger == 'EMERGENCY_HOLD':
            if self.flight_state in (FlightState.HOVERING, FlightState.FLYING):
                self.emergency_hold_active = True
                self._transition_to(FlightState.EMERGENCY_HOLD)

        elif danger == 'CLEAR':
            if self.flight_state == FlightState.EMERGENCY_HOLD:
                self.emergency_hold_active = False
                # Return to HOVERING; if a path exists, state machine
                # will advance to FLYING in the next tick
                self._transition_to(FlightState.HOVERING)

    # =================================================================
    #  MAVLink Communication
    # =================================================================

    def _request_data_streams(self):
        """Ask ArduPilot to send us position / state at reasonable rates."""
        self.mav_conn.mav.request_data_stream_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        self.mav_conn.mav.request_data_stream_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
        self.mav_conn.mav.request_data_stream_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 4, 1)
        self.get_logger().info('Requested data streams from ArduPilot')

    def _mavlink_loop(self):
        """
        High-frequency timer: drain all pending MAVLink messages.
        Updates internal state (connected, armed, mode, position).
        """
        # Heartbeat timeout detection
        if self.last_heartbeat_time is not None:
            elapsed = time.monotonic() - self.last_heartbeat_time
            if elapsed > self.heartbeat_timeout and self.connected:
                self.get_logger().warn(
                    f'Heartbeat timeout ({elapsed:.1f}s) — connection lost')
                self.connected = False

        while True:
            msg = self.mav_conn.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                self.diag_heartbeat_count += 1
                self.connected = True
                self.last_heartbeat_time = time.monotonic()
                self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                mode_num = msg.custom_mode
                self.mode = self.MODE_MAP.get(mode_num, f'MODE_{mode_num}')

                # Detect disarm during landing
                if self.flight_state == FlightState.LANDING and not self.armed:
                    self._transition_to(FlightState.LANDED)

            elif msg_type == 'LOCAL_POSITION_NED':
                # ArduPilot NED → ROS ENU conversion
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = msg.x      # North → X
                pose.pose.position.y = -msg.y     # East (neg) → Y
                pose.pose.position.z = -msg.z     # Down (neg) → Z (up)
                pose.pose.orientation.w = 1.0
                self.current_pose = pose
                self.pose_pub.publish(pose)

                # Detect takeoff completion by altitude
                if self.flight_state == FlightState.TAKING_OFF and \
                   self.armed and self.mode == 'GUIDED' and \
                   (-msg.z) >= self.target_altitude * 0.85:
                    self.get_logger().info(
                        f'Altitude {-msg.z:.2f}m reached — takeoff complete')
                    self._transition_to(FlightState.HOVERING)

                # Check waypoint proximity (only when flying)
                if self.flight_state == FlightState.FLYING and \
                   self.current_path and self.target_setpoint:
                    dist = self._distance(
                        self.current_pose.pose.position,
                        self.target_setpoint.pose.position)
                    if dist < self.waypoint_threshold:
                        self._advance_waypoint()

            elif msg_type == 'COMMAND_ACK':
                self.get_logger().info(
                    f'ACK cmd={msg.command} result={msg.result}')

    def _send_set_mode(self, mode_num: int):
        """Send MAV_CMD_DO_SET_MODE."""
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num,
            0, 0, 0, 0, 0)

    def _send_arm(self):
        """Send arm command."""
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0)
        self.get_logger().info('ARM command sent')

    def _send_takeoff(self, alt: float):
        """Send takeoff command."""
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # min pitch
            0, 0,
            0,  # yaw (NaN = current)
            0, 0,
            alt)
        self.get_logger().info(f'TAKEOFF command sent (alt={alt}m)')

    def _send_land(self):
        """Send land command by switching to LAND mode."""
        self._send_set_mode(self.LAND_MODE)
        self.get_logger().info('LAND mode command sent')

    def _send_position_target(self, x: float, y: float, z: float):
        """
        Send SET_POSITION_TARGET_LOCAL_NED.
        Input is in ENU (ROS convention), converted to NED for ArduPilot.
        """
        # ENU → NED: x_ned = x_enu, y_ned = -y_enu, z_ned = -z_enu
        self.mav_conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (ignored)
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask: position only
            x,        # north
            -y,       # east
            -z,       # down
            0, 0, 0,  # velocity (ignored)
            0, 0, 0,  # acceleration (ignored)
            0, 0)     # yaw, yaw_rate (ignored)

    # =================================================================
    #  Path following
    # =================================================================

    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        self.current_path = msg
        self.current_waypoint_idx = 0
        self.diag_path_count += 1
        self.get_logger().info(f'New path: {len(msg.poses)} waypoints')
        self._update_target()

        # If hovering and a new path arrives, start flying
        if self.flight_state == FlightState.HOVERING:
            self._transition_to(FlightState.FLYING)

    def _update_target(self):
        if self.current_path is None or \
           self.current_waypoint_idx >= len(self.current_path.poses):
            self.target_setpoint = None
            return
        wp = self.current_path.poses[self.current_waypoint_idx]
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = wp.pose.position.x
        sp.pose.position.y = wp.pose.position.y
        sp.pose.position.z = self.target_altitude
        sp.pose.orientation = wp.pose.orientation
        self.target_setpoint = sp
        self.get_logger().info(
            f'Target WP {self.current_waypoint_idx+1}: '
            f'({sp.pose.position.x:.2f}, {sp.pose.position.y:.2f}, '
            f'{sp.pose.position.z:.2f})')

    def _advance_waypoint(self):
        if self.current_path is None:
            return
        self.current_waypoint_idx += 1
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.get_logger().info('Path complete!')
            self.current_path = None
            self.target_setpoint = None
            if self.auto_land_on_complete:
                self.get_logger().info('Auto-landing after path complete')
                self._send_land()
                self._transition_to(FlightState.LANDING)
            else:
                self._transition_to(FlightState.HOVERING)
            return
        self.get_logger().info(
            f'WP reached — advancing to {self.current_waypoint_idx+1}')
        self._update_target()

    def _setpoint_loop(self):
        """Publish position setpoints at 20 Hz (only when airborne)."""
        # Only send setpoints in states where we should be controlling position
        if self.flight_state not in (FlightState.HOVERING, FlightState.FLYING,
                                     FlightState.EMERGENCY_HOLD):
            return

        if self.target_setpoint is not None and \
           self.flight_state == FlightState.FLYING:
            p = self.target_setpoint.pose.position
            self._send_position_target(p.x, p.y, p.z)
            self.diag_setpoint_count += 1
        elif self.current_pose is not None:
            # Hold current position (HOVERING or EMERGENCY_HOLD)
            p = self.current_pose.pose.position
            self._send_position_target(p.x, p.y, p.z)
            self.diag_setpoint_count += 1

    # =================================================================
    #  Status / diagnostics
    # =================================================================

    def _status_loop(self):
        msg = String()
        if not self.connected:
            msg.data = 'Status: CONNECTION_LOST'
        elif self.flight_state == FlightState.EMERGENCY_HOLD:
            msg.data = f'Status: EMERGENCY_HOLD — Armed: {self.armed}'
        elif self.current_path is None:
            msg.data = (f'Status: {self.flight_state} — Mode: {self.mode} '
                        f'— Armed: {self.armed}')
        else:
            total = len(self.current_path.poses)
            msg.data = (f'Status: {self.flight_state} — '
                        f'WP {self.current_waypoint_idx+1}/{total} '
                        f'— Mode: {self.mode} — Armed: {self.armed}')
        self.status_pub.publish(msg)

    def _diagnostics_loop(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('[DIAG] Control Node (PyMAVLink)')
        self.get_logger().info(f'  state={self.flight_state}')
        self.get_logger().info(f'  connected={self.connected}  mode={self.mode}'
                               f'  armed={self.armed}')
        if self.current_pose:
            p = self.current_pose.pose.position
            self.get_logger().info(f'  pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        if self.target_setpoint:
            t = self.target_setpoint.pose.position
            self.get_logger().info(f'  tgt=({t.x:.2f}, {t.y:.2f}, {t.z:.2f})')
        self.get_logger().info(f'  heartbeats={self.diag_heartbeat_count}'
                               f'  setpoints={self.diag_setpoint_count}'
                               f'  paths={self.diag_path_count}')
        self.get_logger().info('=' * 50)

    # =================================================================
    #  Helpers
    # =================================================================

    @staticmethod
    def _distance(a: Point, b: Point) -> float:
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mav_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
