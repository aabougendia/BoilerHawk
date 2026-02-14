#!/usr/bin/env python3
"""
Control Node for BoilerHawk (PyMAVLink version).
Communicates directly with ArduPilot via MAVLink over TCP.
Receives paths from planning module and sends position commands.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from pymavlink import mavutil


class ControlNode(Node):
    """
    ROS 2 node for drone control via direct MAVLink (pymavlink).
    """

    # ArduPilot GUIDED mode number
    GUIDED_MODE = 4
    # Copter modes from ArduPilot
    MODE_MAP = {0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
                5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD'}

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

        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        setpoint_rate = self.get_parameter('setpoint_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_mode_switch = self.get_parameter('auto_mode_switch').value
        self.target_altitude = self.get_parameter('target_altitude').value
        fcu_connection = self.get_parameter('fcu_connection').value
        mavlink_poll_rate = self.get_parameter('mavlink_poll_rate').value

        # ── State variables ──────────────────────────────────────────
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None          # PoseStamped (NED → ENU converted)
        self.target_setpoint = None       # PoseStamped

        self.connected = False
        self.armed = False
        self.mode = 'UNKNOWN'
        self.takeoff_requested = False
        self.takeoff_complete = False

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
        self.get_logger().info(
            f'Heartbeat received (system {self.mav_conn.target_system} '
            f'comp {self.mav_conn.target_component})'
        )

        # Request data streams from ArduPilot (position, attitude, etc.)
        self._request_data_streams()

        # ── ROS 2 Subscribers ────────────────────────────────────────
        self.path_sub = self.create_subscription(
            Path, '/local_path', self.path_callback, 10)

        # ── ROS 2 Publishers ─────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/control/status', 10)
        # Publish pose so RViz / other nodes can use it
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavlink/local_position/pose', 10)

        # ── Timers ───────────────────────────────────────────────────
        # MAVLink receive loop (high frequency)
        self.create_timer(1.0 / mavlink_poll_rate, self._mavlink_loop)
        # Setpoint publishing
        self.create_timer(1.0 / setpoint_rate, self._setpoint_loop)
        # Status & diagnostics
        self.create_timer(1.0, self._status_loop)
        self.create_timer(5.0, self._diagnostics_loop)
        # Auto-control state machine (arm / mode / takeoff)
        self.create_timer(2.0, self._auto_control_loop)

        self.get_logger().info('Control node initialized (PyMAVLink)')
        self.get_logger().info(f'  waypoint_threshold={self.waypoint_threshold}m')
        self.get_logger().info(f'  setpoint_rate={setpoint_rate}Hz')
        self.get_logger().info(f'  auto_arm={self.auto_arm}')
        self.get_logger().info(f'  target_altitude={self.target_altitude}m')

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
        while True:
            msg = self.mav_conn.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                self.diag_heartbeat_count += 1
                self.connected = True
                self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                mode_num = msg.custom_mode
                self.mode = self.MODE_MAP.get(mode_num, f'MODE_{mode_num}')

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
                if not self.takeoff_complete and self.armed and \
                   self.mode == 'GUIDED' and (-msg.z) >= self.target_altitude * 0.85:
                    self.get_logger().info(
                        f'Altitude {-msg.z:.2f}m reached — takeoff complete')
                    self.takeoff_complete = True

                # Check waypoint proximity
                if self.current_path and self.target_setpoint:
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
    #  Auto-control state machine
    # =================================================================

    def _auto_control_loop(self):
        """
        Runs every 2s. Drives the arm → mode → takeoff sequence.
        """
        if not self.connected:
            return

        # Step 1: Set GUIDED mode
        if self.auto_mode_switch and self.mode != 'GUIDED':
            self.get_logger().info(f'Current mode: {self.mode} → requesting GUIDED')
            self._send_set_mode(self.GUIDED_MODE)
            return

        # Step 2: Arm
        if self.auto_arm and not self.armed and self.mode == 'GUIDED':
            self.get_logger().info('Requesting ARM')
            self._send_arm()
            return

        # Step 3: Takeoff
        if self.armed and self.mode == 'GUIDED' and not self.takeoff_requested:
            self.get_logger().info('Requesting TAKEOFF')
            self._send_takeoff(self.target_altitude)
            self.takeoff_requested = True

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
            self.get_logger().info('Path complete! Holding final position.')
            return
        self.get_logger().info(
            f'WP reached — advancing to {self.current_waypoint_idx+1}')
        self._update_target()

    def _setpoint_loop(self):
        """Publish position setpoints at 20 Hz (only after takeoff)."""
        if not self.takeoff_complete:
            return
        if self.target_setpoint is not None:
            p = self.target_setpoint.pose.position
            self._send_position_target(p.x, p.y, p.z)
            self.diag_setpoint_count += 1
        elif self.current_pose is not None:
            # Hold current position
            p = self.current_pose.pose.position
            self._send_position_target(p.x, p.y, p.z)
            self.diag_setpoint_count += 1

    # =================================================================
    #  Status / diagnostics
    # =================================================================

    def _status_loop(self):
        msg = String()
        if not self.connected:
            msg.data = 'Status: Waiting for MAVLink connection'
        elif self.current_path is None:
            msg.data = (f'Status: No path — Mode: {self.mode} '
                        f'— Armed: {self.armed}')
        else:
            total = len(self.current_path.poses)
            msg.data = (f'Status: Following — WP {self.current_waypoint_idx+1}'
                        f'/{total} — Mode: {self.mode} — Armed: {self.armed}')
        self.status_pub.publish(msg)

    def _diagnostics_loop(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('[DIAG] Control Node (PyMAVLink)')
        self.get_logger().info(f'  connected={self.connected}  mode={self.mode}'
                               f'  armed={self.armed}')
        self.get_logger().info(f'  takeoff_req={self.takeoff_requested}'
                               f'  takeoff_done={self.takeoff_complete}')
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
