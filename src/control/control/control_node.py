#!/usr/bin/env python3
"""
Control Node for BoilerHawk — VFH + A* + Velocity Setpoints.

Single-node architecture:
  Occupancy Grid → A* path planner → VFH local avoidance → EMA filter → velocity → ArduPilot
  Also sends OBSTACLE_DISTANCE as a secondary BendyRuler safety layer.
"""

import math
import time
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from pymavlink import mavutil


class ControlNode(Node):
    """
    ROS 2 node: runs VFH internally on occupancy grid, outputs velocity
    setpoints to ArduPilot.  Also feeds OBSTACLE_DISTANCE for BendyRuler.
    """

    GUIDED_MODE = 4
    MODE_MAP = {0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
                5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD'}

    # OBSTACLE_DISTANCE sectors
    OBS_SECTORS = 72
    OBS_SECTOR_DEG = 360.0 / 72  # 5°

    # VFH sectors (matching planning package)
    VFH_SECTORS = 72

    def __init__(self):
        super().__init__('control_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('setpoint_rate', 20.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_mode_switch', True)
        self.declare_parameter('target_altitude', 1.0)
        self.declare_parameter('fcu_connection', 'tcp:127.0.0.1:5760')
        self.declare_parameter('mavlink_poll_rate', 50.0)
        self.declare_parameter('occupancy_topic', '/perception/occupancy')
        self.declare_parameter('obstacle_distance_rate', 10.0)
        self.declare_parameter('avoid_only', True)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', -20.0)
        self.declare_parameter('goal_reached_threshold', 1.0)

        # VFH params
        self.declare_parameter('safety_radius_cells', 3)
        self.declare_parameter('valley_threshold', 0.5)

        # Speed tiers
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('turn_speed_medium', 0.8)
        self.declare_parameter('turn_speed_sharp', 0.3)

        # EMA heading filter
        self.declare_parameter('heading_ema_alpha', 0.3)

        # A* path planner
        self.declare_parameter('astar_lookahead_cells', 15)

        # Read params
        setpoint_rate = self.get_parameter('setpoint_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_mode_switch = self.get_parameter('auto_mode_switch').value
        self.target_altitude = self.get_parameter('target_altitude').value
        self.fcu_connection = self.get_parameter('fcu_connection').value
        mavlink_poll_rate = self.get_parameter('mavlink_poll_rate').value
        occupancy_topic = self.get_parameter('occupancy_topic').value
        obstacle_distance_rate = self.get_parameter('obstacle_distance_rate').value
        self.avoid_only = self.get_parameter('avoid_only').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_reached_threshold = self.get_parameter('goal_reached_threshold').value
        self.safety_radius_cells = self.get_parameter('safety_radius_cells').value
        self.valley_threshold = self.get_parameter('valley_threshold').value
        self.max_speed = self.get_parameter('max_speed').value
        self.turn_speed_medium = self.get_parameter('turn_speed_medium').value
        self.turn_speed_sharp = self.get_parameter('turn_speed_sharp').value
        self.heading_ema_alpha = self.get_parameter('heading_ema_alpha').value
        self.astar_lookahead = self.get_parameter('astar_lookahead_cells').value

        # ── State ────────────────────────────────────────────────────
        self.current_pose = None
        self.current_yaw = 0.0  # NED yaw (rad)
        self.connected = False
        self.armed = False
        self.mode = 'UNKNOWN'
        self._prev_mode = 'UNKNOWN'
        self.takeoff_requested = False
        self.takeoff_complete = False
        self._goal_reached = False  # T018: true when within threshold of goal

        # Occupancy grid
        self._occ_grid = None
        self._occ_resolution = 0.1
        self._occ_width = 0
        self._occ_height = 0
        self._last_grid_time = 0.0  # monotonic time of last grid received

        # VFH state
        self._vfh_heading = 0.0       # current filtered heading (body-frame rad)
        self._last_vfh_heading = None  # for EMA filter
        self._vfh_sector_width = 2.0 * math.pi / self.VFH_SECTORS
        self._nearest_obstacle_m = float('inf')  # distance to nearest in chosen dir
        self._all_blocked = False  # True if VFH found no free direction

        # Camera FOV (horizontal half-angle in radians)
        # ~87° total → ±43.5° from forward
        self._camera_fov_half_rad = math.radians(43.5)

        # A* state
        self._astar_path = []  # list of (row, col) from drone center to goal
        self._astar_preferred = 0.0  # body-frame angle from A* path

        # Counters
        self.diag_heartbeat_count = 0
        self.diag_msg_count = 0
        self.diag_setpoint_count = 0
        self.diag_obs_dist_count = 0
        self.diag_vfh_count = 0
        self.last_heartbeat_time = 0.0
        self.streams_requested = False

        # ── MAVLink ──────────────────────────────────────────────────
        self.mav_conn = None
        self._connect()

        # ── ROS 2 ────────────────────────────────────────────────────
        self.create_subscription(
            OccupancyGrid, occupancy_topic, self._occupancy_callback, 10)
        self.status_pub = self.create_publisher(String, '/control/status', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavlink/local_position/pose', 10)

        # ── Timers ───────────────────────────────────────────────────
        self.create_timer(1.0 / mavlink_poll_rate, self._mavlink_loop)
        self.create_timer(1.0 / setpoint_rate, self._setpoint_loop)
        self.create_timer(1.0 / obstacle_distance_rate, self._obstacle_distance_loop)
        self.create_timer(1.0, self._status_loop)
        self.create_timer(5.0, self._diagnostics_loop)
        self.create_timer(2.0, self._auto_control_loop)
        self.create_timer(3.0, self._connection_health_loop)

        self.get_logger().info('Control node initialized (VFH + Velocity)')
        self.get_logger().info(f'  avoid_only={self.avoid_only}  '
                               f'alt={self.target_altitude}m')
        self.get_logger().info(f'  speeds: max={self.max_speed} med={self.turn_speed_medium} '
                               f'sharp={self.turn_speed_sharp} m/s')

    # =================================================================
    #  Connection Management
    # =================================================================

    def _connect(self):
        self.get_logger().info(f'Connecting to FCU: {self.fcu_connection}')
        try:
            self.mav_conn = mavutil.mavlink_connection(
                self.fcu_connection, source_system=255, source_component=0)
            self.mav_conn.target_system = 1
            self.mav_conn.target_component = 1
            hb = self.mav_conn.wait_heartbeat(timeout=5)
            if hb is not None:
                self.connected = True
                self.last_heartbeat_time = time.monotonic()
                self.get_logger().info(
                    f'Heartbeat received (src_sys={hb.get_srcSystem()})')
                self.mav_conn.target_system = 1
                self.mav_conn.target_component = 1
            else:
                self.get_logger().warn('No heartbeat yet')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')

    def _connection_health_loop(self):
        if self.mav_conn is None:
            self._connect()
            return
        now = time.monotonic()
        since_hb = now - self.last_heartbeat_time if self.last_heartbeat_time > 0 else 999
        if since_hb > 5.0 or not self.streams_requested:
            self._request_data_streams()
            self.streams_requested = True
        if since_hb > 30.0:
            self.get_logger().warn('No heartbeat — reconnecting')
            self.connected = False
            try:
                self.mav_conn.close()
            except Exception:
                pass
            self._connect()

    # =================================================================
    #  MAVLink Communication
    # =================================================================

    def _request_data_streams(self):
        if self.mav_conn is None:
            return
        self.mav_conn.mav.request_data_stream_send(
            1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    def _mavlink_loop(self):
        if self.mav_conn is None:
            return
        try:
            for _ in range(50):
                msg = self.mav_conn.recv_match(blocking=False)
                if msg is None:
                    break
                msg_type = msg.get_type()
                if msg_type == 'BAD_DATA':
                    continue
                self.diag_msg_count += 1

                if msg_type == 'HEARTBEAT':
                    if msg.get_srcSystem() == 0:
                        continue
                    self.diag_heartbeat_count += 1
                    self.connected = True
                    self.last_heartbeat_time = time.monotonic()
                    self.armed = (msg.base_mode &
                                  mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    self.mode = self.MODE_MAP.get(msg.custom_mode,
                                                  f'MODE_{msg.custom_mode}')

                elif msg_type == 'LOCAL_POSITION_NED':
                    pose = PoseStamped()
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = msg.x       # North
                    pose.pose.position.y = -msg.y      # East→Y (ENU)
                    pose.pose.position.z = -msg.z      # Down→Z up (ENU)
                    pose.pose.orientation.w = 1.0
                    self.current_pose = pose
                    self.pose_pub.publish(pose)

                    if not self.takeoff_complete and self.armed and \
                       self.mode == 'GUIDED' and \
                       (-msg.z) >= self.target_altitude * 0.85:
                        self.get_logger().info(
                            f'Altitude {-msg.z:.2f}m — takeoff complete')
                        self.takeoff_complete = True

                elif msg_type == 'ATTITUDE':
                    self.current_yaw = msg.yaw  # NED yaw (rad)

                elif msg_type == 'COMMAND_ACK':
                    self.get_logger().info(
                        f'ACK cmd={msg.command} result={msg.result}')

        except Exception as e:
            self.get_logger().warn(f'MAVLink recv error: {e}')

    def _send_set_mode(self, mode_num):
        if self.mav_conn is None:
            return
        self.mav_conn.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_num,
            0, 0, 0, 0, 0)

    def _send_arm(self):
        if self.mav_conn is None:
            return
        self.mav_conn.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        self.get_logger().info('ARM command sent')

    def _send_takeoff(self, alt):
        if self.mav_conn is None:
            return
        self.mav_conn.mav.command_long_send(
            1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, alt)
        self.get_logger().info(f'TAKEOFF command sent (alt={alt}m)')

    def _send_velocity_ned(self, vn, ve, vd):
        """Send velocity setpoint in NED frame."""
        if self.mav_conn is None:
            return
        # type_mask: ignore position (0b111), use velocity (0b000),
        # ignore accel (0b111), ignore yaw/yaw_rate (0b11)
        self.mav_conn.mav.set_position_target_local_ned_send(
            0, 1, 1,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # velocity only
            0, 0, 0,            # pos (ignored)
            vn, ve, vd,         # vel NED
            0, 0, 0,            # accel (ignored)
            0, 0)               # yaw, yaw_rate (ignored)

    # =================================================================
    #  VFH Direction Picker (embedded, no cross-package dependency)
    # =================================================================

    def _run_vfh(self, preferred_angle: float) -> float:
        """
        Build polar histogram from occupancy grid, find best free direction.
        All angles in radians, 0 = forward (+X in grid = body forward).
        Returns chosen heading in body frame.
        """
        if self._occ_grid is None:
            return preferred_angle

        grid = self._occ_grid
        h, w = grid.shape
        cx, cy = w // 2, h // 2
        res = self._occ_resolution

        # ── inflate obstacles ──
        occupied = grid >= 50
        r = self.safety_radius_cells
        if r > 0:
            inflated = np.zeros_like(occupied)
            occ_rows, occ_cols = np.where(occupied)
            for oi, oj in zip(occ_rows, occ_cols):
                r0, r1 = max(oi - r, 0), min(oi + r + 1, h)
                c0, c1 = max(oj - r, 0), min(oj + r + 1, w)
                inflated[r0:r1, c0:c1] = True
        else:
            inflated = occupied

        # ── build polar histogram (obstacle density per sector) ──
        n = self.VFH_SECTORS
        histogram = np.zeros(n, dtype=float)

        occ_r, occ_c = np.where(inflated)
        if len(occ_r) == 0:
            return preferred_angle  # no obstacles — go preferred

        dr = occ_r - cy  # row offset (positive = down in grid)
        dc = occ_c - cx  # col offset (positive = right = forward in body)
        dist = np.sqrt(dr.astype(float)**2 + dc.astype(float)**2) * res
        dist = np.maximum(dist, 0.01)

        # Angle: atan2(-dr, dc) gives angle from forward (+col direction)
        # In the grid: +col = forward (camera X), +row = right/down
        # This matches the original VFH planner convention
        angles = np.arctan2(-dr.astype(float), dc.astype(float))
        angles = np.mod(angles, 2.0 * math.pi)

        sectors = (angles / self._vfh_sector_width).astype(int) % n
        weights = 1.0 / dist  # closer obstacles weigh more

        for s, w_val in zip(sectors, weights):
            histogram[s] += w_val

        # ── T004: FOV-aware sector masking ──
        # Mark sectors outside camera FOV as blocked
        fov_half = self._camera_fov_half_rad
        for s in range(n):
            sector_angle = (s + 0.5) * self._vfh_sector_width  # 0..2π
            # Normalize to [-π, π] from forward
            angle_from_fwd = ((sector_angle + math.pi) % (2 * math.pi)) - math.pi
            if abs(angle_from_fwd) > fov_half:
                histogram[s] = 999.0  # mark as blocked (unknown region)

        # ── find best free direction ──
        threshold = self.valley_threshold
        free = histogram < threshold

        # Find free valleys (contiguous runs of free sectors)
        valleys = []
        extended = np.concatenate([free, free])
        start = None
        length = 0
        for i in range(2 * n):
            if extended[i]:
                if start is None:
                    start = i % n
                length += 1
            else:
                if start is not None and length > 0:
                    valleys.append((start, min(length, n)))
                start = None
                length = 0
        if start is not None and length > 0:
            valleys.append((start, min(length, n)))

        if not valleys:
            # T007: All blocked — signal hover
            self._all_blocked = True
            self._nearest_obstacle_m = 0.0
            return preferred_angle  # heading doesn't matter, will hover

        self._all_blocked = False

        # Pick valley that puts us closest to preferred angle,
        # but prefer the edge nearest to preferred (not center)
        pref = np.mod(preferred_angle, 2.0 * math.pi)
        best_angle = preferred_angle
        best_cost = float('inf')

        for v_start, v_len in valleys:
            # Check edge closest to preferred, and center
            candidates = [
                v_start,
                (v_start + v_len - 1) % n,
                (v_start + v_len // 2) % n,
            ]
            for c in candidates:
                angle = (c + 0.5) * self._vfh_sector_width
                diff = abs(((angle - pref) + math.pi) % (2 * math.pi) - math.pi)
                if diff < best_cost:
                    best_cost = diff
                    best_angle = angle

        # ── T005: compute nearest obstacle distance in chosen direction ──
        chosen_sector = int((best_angle / self._vfh_sector_width)) % n
        # Raycast along chosen heading to find nearest obstacle
        cos_a = math.cos(best_angle)
        sin_a = math.sin(best_angle)  # sin component is -dr direction
        nearest = float('inf')
        max_steps = min(cx, cy)
        for step in range(1, max_steps):
            col = int(round(cx + step * cos_a))
            row = int(round(cy - step * sin_a))  # -sin because atan2(-dr,...)
            if row < 0 or row >= h or col < 0 or col >= w:
                break
            if grid[row, col] >= 50:  # use original grid, not inflated
                nearest = step * res
                break
        self._nearest_obstacle_m = nearest

        return best_angle

    # =================================================================
    #  A* Path Planner (embedded, lightweight)
    # =================================================================

    def _run_astar(self, grid: np.ndarray, start: tuple, goal: tuple,
                   max_iterations: int = 3000) -> list:
        """
        Lightweight A* on the occupancy grid.
        Args:
            grid: 2D numpy array (>=50 = obstacle)
            start: (row, col) — typically grid center
            goal: (row, col) — target cell
            max_iterations: cap to prevent slow searches
        Returns:
            List of (row, col) from start to goal, or [] if no path.
        """
        h, w = grid.shape

        def valid(r, c):
            return 0 <= r < h and 0 <= c < w and grid[r, c] < 50

        if not valid(*start) or not valid(*goal):
            return []

        # Node: (f, g, row, col, parent_row, parent_col)
        open_heap = []
        g_start = 0.0
        h_start = math.hypot(goal[0] - start[0], goal[1] - start[1])
        heapq.heappush(open_heap, (h_start, g_start, start[0], start[1], -1, -1))

        came_from = {}  # (r,c) -> (pr, pc)
        g_score = {start: 0.0}
        closed = set()

        # 8-connected neighbors
        dirs = [(-1, -1), (-1, 0), (-1, 1),
                (0, -1),           (0, 1),
                (1, -1),  (1, 0),  (1, 1)]

        iterations = 0
        while open_heap and iterations < max_iterations:
            iterations += 1
            f, g, r, c, pr, pc = heapq.heappop(open_heap)
            pos = (r, c)

            if pos in closed:
                continue
            closed.add(pos)
            if pr >= 0:
                came_from[pos] = (pr, pc)

            # goal reached (within 1 cell)
            if abs(r - goal[0]) <= 1 and abs(c - goal[1]) <= 1:
                # reconstruct path
                path = [pos]
                while pos in came_from:
                    pos = came_from[pos]
                    path.append(pos)
                path.reverse()
                return path

            for dr, dc in dirs:
                nr, nc = r + dr, c + dc
                npos = (nr, nc)
                if npos in closed or not valid(nr, nc):
                    continue
                move_cost = 1.414 if (abs(dr) + abs(dc)) == 2 else 1.0
                tentative_g = g + move_cost
                if tentative_g < g_score.get(npos, float('inf')):
                    g_score[npos] = tentative_g
                    h_val = math.hypot(goal[0] - nr, goal[1] - nc)
                    heapq.heappush(open_heap, (tentative_g + h_val, tentative_g,
                                               nr, nc, r, c))

        return []  # no path found

    def _astar_preferred_angle(self, grid: np.ndarray, goal_cell: tuple) -> float:
        """
        Run A* from grid center to goal_cell, return body-frame angle
        toward the next waypoint on the path (a few cells ahead).
        """
        h, w = grid.shape
        cx, cy = w // 2, h // 2
        start = (cy, cx)  # (row, col)

        # Inflate grid for A* path safety
        occupied = grid >= 50
        r = self.safety_radius_cells
        if r > 0:
            inflated = np.zeros_like(occupied)
            occ_rows, occ_cols = np.where(occupied)
            for oi, oj in zip(occ_rows, occ_cols):
                r0, r1 = max(oi - r, 0), min(oi + r + 1, h)
                c0, c1 = max(oj - r, 0), min(oj + r + 1, w)
                inflated[r0:r1, c0:c1] = True
            safe_grid = np.where(inflated, 100, grid)
        else:
            safe_grid = grid

        path = self._run_astar(safe_grid, start, goal_cell)
        self._astar_path = path

        if len(path) < 2:
            return 0.0  # no path or already at goal — default forward

        # Pick a waypoint a few cells ahead for smoother steering
        lookahead_idx = min(self.astar_lookahead, len(path) - 1)
        wp_row, wp_col = path[lookahead_idx]

        # Convert grid offset to body-frame angle
        # +col = forward (camera X), -row offset = right in body frame
        dc = wp_col - cx
        dr = wp_row - cy
        angle = math.atan2(-dr, dc)  # same convention as VFH
        return angle

    # =================================================================
    #  Occupancy Grid Callback
    # =================================================================

    def _occupancy_callback(self, msg: OccupancyGrid):
        """Store grid, run A* for direction, then VFH for safe avoidance."""
        w = msg.info.width
        h = msg.info.height
        self._occ_grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self._occ_resolution = msg.info.resolution
        self._occ_width = w
        self._occ_height = h
        self._last_grid_time = time.monotonic()

        cx, cy = w // 2, h // 2

        if self.avoid_only:
            # ── Avoid-only mode: A* to far forward edge ──
            # Goal = center of the far forward column edge
            goal_cell = (cy, w - 2)  # (row, col): same row as center, far right col = forward
            preferred = self._astar_preferred_angle(self._occ_grid, goal_cell)
        else:
            # ── Goal mode: A* toward goal position ──
            if self.current_pose is not None:
                p = self.current_pose.pose.position
                dx = self.goal_x - p.x   # world-frame forward offset
                dy = self.goal_y - p.y   # world-frame lateral offset

                # Rotate world offset into body frame using current yaw
                cos_y = math.cos(-self.current_yaw)
                sin_y = math.sin(-self.current_yaw)
                body_fwd = dx * cos_y - dy * sin_y   # body forward
                body_lat = dx * sin_y + dy * cos_y   # body left

                # Convert body-frame meters to grid cell offset
                res = self._occ_resolution
                goal_col = int(cx + body_fwd / res)
                goal_row = int(cy - body_lat / res)  # -lat because +row = down

                # Clamp to grid bounds (leave 1-cell border)
                goal_col = max(1, min(w - 2, goal_col))
                goal_row = max(1, min(h - 2, goal_row))

                preferred = self._astar_preferred_angle(self._occ_grid,
                                                        (goal_row, goal_col))
            else:
                preferred = 0.0
                self._astar_path = []

        raw_heading = self._run_vfh(preferred)

        # EMA filter
        if self._last_vfh_heading is not None:
            diff = ((raw_heading - self._last_vfh_heading) + math.pi) % (2 * math.pi) - math.pi
            filtered = self._last_vfh_heading + self.heading_ema_alpha * diff
        else:
            filtered = raw_heading

        self._vfh_heading = filtered
        self._last_vfh_heading = filtered
        self.diag_vfh_count += 1

    # =================================================================
    #  OBSTACLE_DISTANCE (secondary BendyRuler safety layer)
    # =================================================================

    def _grid_to_sector_distances(self):
        if self._occ_grid is None:
            return [65535] * self.OBS_SECTORS

        grid = self._occ_grid
        h, w = grid.shape
        cx, cy = w // 2, h // 2
        res = self._occ_resolution
        max_range_cells = min(cx, cy)

        distances = [65535] * self.OBS_SECTORS
        for sector in range(self.OBS_SECTORS):
            angle_rad = math.radians(sector * self.OBS_SECTOR_DEG)
            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)
            for step in range(1, max_range_cells):
                col = int(round(cx + step * dx))
                row = int(round(cy + step * dy))
                if row < 0 or row >= h or col < 0 or col >= w:
                    break
                if grid[row, col] >= 50:
                    distances[sector] = int(step * res * 100)
                    break
        return distances

    def _obstacle_distance_loop(self):
        if self.mav_conn is None or not self.connected:
            return
        distances = self._grid_to_sector_distances()
        max_dist = int(min(self._occ_width, self._occ_height) // 2
                       * self._occ_resolution * 100)
        self.mav_conn.mav.obstacle_distance_send(
            int(time.time() * 1e6), 0, distances,
            int(self.OBS_SECTOR_DEG), 20, max(max_dist, 100),
            0.0, 0.0, mavutil.mavlink.MAV_FRAME_BODY_FRD)
        self.diag_obs_dist_count += 1

    # =================================================================
    #  Auto-control state machine
    # =================================================================

    def _auto_control_loop(self):
        if not self.connected:
            return
        if self.auto_mode_switch and self.mode != 'GUIDED':
            self.get_logger().info(f'Mode: {self.mode} → requesting GUIDED')
            self._send_set_mode(self.GUIDED_MODE)
            return
        if self.auto_arm and not self.armed and self.mode == 'GUIDED':
            self.get_logger().info('Requesting ARM')
            self._send_arm()
            return
        if self.armed and self.mode == 'GUIDED' and not self.takeoff_requested:
            self.get_logger().info('Requesting TAKEOFF')
            self._send_takeoff(self.target_altitude)
            self.takeoff_requested = True

    # =================================================================
    #  Velocity Setpoint Loop
    # =================================================================

    def _speed_for_steer(self, steer_deg: float) -> float:
        a = abs(steer_deg)
        if a < 15.0:
            return self.max_speed
        elif a < 35.0:
            return self.turn_speed_medium
        else:
            return self.turn_speed_sharp

    def _setpoint_loop(self):
        """
        Convert VFH heading to velocity setpoint and send at 20 Hz.
        VFH heading is in body frame; rotate by current yaw to get NED.
        Includes emergency stop, hover-when-blocked, and sensor timeout.
        """
        if not self.takeoff_complete or self.current_pose is None:
            return

        # ── altitude correction (always active) ──
        alt_error = self.target_altitude - self.current_pose.pose.position.z
        vd = -alt_error * 1.0  # NED: negative = up

        # ── T008: sensor timeout safety ──
        grid_age = time.monotonic() - self._last_grid_time if self._last_grid_time > 0 else 999
        if grid_age > 2.0:
            self.get_logger().warn('Hovering — no sensor data (%.1fs)', grid_age)
            self._send_velocity_ned(0.0, 0.0, vd)
            self.diag_setpoint_count += 1
            return

        # ── T007: hover when all directions blocked ──
        if self._all_blocked:
            self.get_logger().warn('Hovering — all directions blocked')
            self._send_velocity_ned(0.0, 0.0, vd)
            self.diag_setpoint_count += 1
            return

        # ── T006: emergency stop if nearest obstacle ≤1.5m ──
        if self._nearest_obstacle_m <= 1.5:
            self._send_velocity_ned(0.0, 0.0, vd)
            self.diag_setpoint_count += 1
            return

        # ── T018: goal-reached detection ──
        if not self.avoid_only and self.current_pose is not None:
            p = self.current_pose.pose.position
            dist_to_goal = math.sqrt((self.goal_x - p.x)**2 + (self.goal_y - p.y)**2)
            if dist_to_goal < self.goal_reached_threshold:
                if not self._goal_reached:
                    self.get_logger().info(
                        f'Goal reached! dist={dist_to_goal:.2f}m — hovering')
                    self._goal_reached = True
                self._send_velocity_ned(0.0, 0.0, vd)
                self.diag_setpoint_count += 1
                return
            else:
                self._goal_reached = False

        # ── VFH heading → velocity ──
        heading_body = self._vfh_heading  # body-frame angle

        # Steer angle = how far from straight ahead
        steer_rad = ((heading_body) + math.pi) % (2 * math.pi) - math.pi
        steer_deg = math.degrees(steer_rad)

        # T006: scale speed by distance to obstacle (smooth transition)
        if self._nearest_obstacle_m <= 5.0:
            speed = self._speed_for_steer(steer_deg)
        else:
            speed = self.max_speed

        # Convert body-frame heading to NED velocity
        # heading_body: 0 = forward in body frame
        # current_yaw: NED yaw of the drone
        ned_heading = self.current_yaw + heading_body

        vn = speed * math.cos(ned_heading)  # North
        ve = speed * math.sin(ned_heading)  # East

        self._send_velocity_ned(vn, ve, vd)
        self.diag_setpoint_count += 1

    # =================================================================
    #  Status / diagnostics
    # =================================================================

    def _status_loop(self):
        msg = String()
        if not self.connected:
            msg.data = 'Status: Waiting for MAVLink'
        elif not self.takeoff_complete:
            msg.data = f'Status: Takeoff — {self.mode} — Armed: {self.armed}'
        else:
            msg.data = (f'Status: Flying (VFH+vel) — {self.mode} — '
                        f'heading={math.degrees(self._vfh_heading):.0f}°')
        self.status_pub.publish(msg)

    def _diagnostics_loop(self):
        now = time.monotonic()
        since_hb = now - self.last_heartbeat_time if self.last_heartbeat_time > 0 else -1
        self.get_logger().info('=' * 50)
        self.get_logger().info('[DIAG] VFH + Velocity Control')
        self.get_logger().info(f'  connected={self.connected}  mode={self.mode}'
                               f'  armed={self.armed}')
        if self.mode != self._prev_mode:
            self.get_logger().warn(
                f'  MODE CHANGE: {self._prev_mode} → {self.mode}')
            self._prev_mode = self.mode
        # T021: enhanced diagnostics
        steer_rad = ((self._vfh_heading) + math.pi) % (2 * math.pi) - math.pi
        steer_deg = math.degrees(steer_rad)
        speed = self._speed_for_steer(steer_deg) if self._nearest_obstacle_m <= 5.0 else self.max_speed
        self.get_logger().info(f'  takeoff={self.takeoff_complete}'
                               f'  heading={math.degrees(self._vfh_heading):.0f}°'
                               f'  steer={steer_deg:.0f}°'
                               f'  speed={speed:.1f}m/s')
        self.get_logger().info(f'  nearest_obs={self._nearest_obstacle_m:.1f}m'
                               f'  all_blocked={self._all_blocked}'
                               f'  astar_path_len={len(self._astar_path)}')
        if self.current_pose:
            p = self.current_pose.pose.position
            self.get_logger().info(f'  pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        self.get_logger().info(
            f'  vfh_runs={self.diag_vfh_count}  obs_dist={self.diag_obs_dist_count}'
            f'  setpoints={self.diag_setpoint_count}')
        self.get_logger().info(f'  heartbeats={self.diag_heartbeat_count}'
                               f'  msgs={self.diag_msg_count}'
                               f'  last_hb={since_hb:.1f}s ago')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.mav_conn is not None:
            node.mav_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
