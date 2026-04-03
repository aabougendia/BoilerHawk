#!/usr/bin/env python3
"""Perception node — persistent world-frame occupancy grid.

Depth-camera point clouds are transformed into world coordinates and
rasterised into a FIXED occupancy grid that accumulates detections.
Ground points are filtered by computing each point's world-Z height.

Auto-detects whether the point cloud is in **optical** frame
(x=right, y=down, z=depth) or **sensor-link** frame
(x=forward, y=left, z=up) by checking which axis has the largest
values on the first cloud received.
"""

import logging
import os
import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid


def _setup_file_logger(node_name: str) -> logging.Logger:
    """Create a Python file logger that writes to logs/<node_name>.log."""
    log_dir = os.path.expanduser('~/BoilerHawk/BoilerHawk/logs')
    os.makedirs(log_dir, exist_ok=True)
    flog = logging.getLogger(f'bhawk.{node_name}')
    flog.setLevel(logging.DEBUG)
    if not flog.handlers:
        fh = logging.FileHandler(
            os.path.join(log_dir, f'{node_name}.log'), mode='w')  # truncate
        fh.setFormatter(logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%H:%M:%S'))
        flog.addHandler(fh)
    return flog


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # --- Parameters ---
        self.declare_parameter('pointcloud_topic', '/depth_camera/points')
        self.declare_parameter('occupancy_topic', '/occupancy_grid')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('resolution', 0.2)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('inflate_radius', 0.6)
        self.declare_parameter('grid_frame', 'map')
        self.declare_parameter('grid_origin_x', -5.0)
        self.declare_parameter('grid_origin_y', -2.0)
        self.declare_parameter('grid_width_m', 10.0)
        self.declare_parameter('grid_height_m', 14.0)
        self.declare_parameter('cam_pitch', 0.2)
        self.declare_parameter('cam_z_offset', 0.05)
        self.declare_parameter('min_obstacle_height', 0.5)
        self.declare_parameter('min_hits', 5)
        self.declare_parameter('hit_increment', 2)
        self.declare_parameter('hit_decay', 1)

        self.pointcloud_topic = str(
            self.get_parameter('pointcloud_topic').value)
        self.occupancy_topic = str(
            self.get_parameter('occupancy_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.inflate_radius = float(
            self.get_parameter('inflate_radius').value)
        self.grid_frame = str(self.get_parameter('grid_frame').value)
        self.grid_origin_x = float(
            self.get_parameter('grid_origin_x').value)
        self.grid_origin_y = float(
            self.get_parameter('grid_origin_y').value)
        self.grid_width_m = float(
            self.get_parameter('grid_width_m').value)
        self.grid_height_m = float(
            self.get_parameter('grid_height_m').value)
        self.cam_pitch = float(self.get_parameter('cam_pitch').value)
        self.cam_z_offset = float(
            self.get_parameter('cam_z_offset').value)
        self.min_obstacle_height = float(
            self.get_parameter('min_obstacle_height').value)
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.hit_increment = int(self.get_parameter('hit_increment').value)
        self.hit_decay = int(self.get_parameter('hit_decay').value)

        # Grid dimensions (fixed, world-frame)
        self.grid_w = int(self.grid_width_m / self.resolution)
        self.grid_h = int(self.grid_height_m / self.resolution)

        # Hit-count grid: each cell counts how many frames confirm it.
        # Only cells with >= min_hits become occupied. This prevents
        # single-frame noise from permanently polluting the grid.
        # Once a cell reaches min_hits it becomes a permanent obstacle
        # and is never decayed — walls don't move.
        self.hit_count = np.zeros(
            (self.grid_h, self.grid_w), dtype=np.int16)

        # --- File logger ---
        self.flog = _setup_file_logger('perception')
        self.flog.info('=== Perception node started ===')

        # --- Drone pose ---
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.drone_yaw = 0.0
        # Full rotation matrix (body → world), 3×3
        self._R_body_to_world = np.eye(3)
        self._have_pose = False

        # --- Frame auto-detection ---
        self._frame = None   # 'optical' or 'sensor_link'
        self._debug_count = 0

        # --- Subscriptions ---
        self.create_subscription(
            PointCloud2, self.pointcloud_topic,
            self.pointcloud_callback, 10)

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            PoseStamped, self.pose_topic, self._pose_cb, mavros_qos)

        # --- Publisher ---
        self.map_pub = self.create_publisher(
            OccupancyGrid, self.occupancy_topic, 10)

        self._last_log_time = None
        init_msg = (
            f'Perception: {self.grid_w}x{self.grid_h} grid, '
            f'res={self.resolution}m, inflate={self.inflate_radius}m, '
            f'min_hits={self.min_hits}, '
            f'hit_inc={self.hit_increment}, decay={self.hit_decay}, '
            f'min_obs_h={self.min_obstacle_height}')
        self.get_logger().info(init_msg)
        self.flog.info(init_msg)

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        self.drone_x = msg.pose.position.x
        self.drone_y = msg.pose.position.y
        self.drone_z = msg.pose.position.z
        q = msg.pose.orientation
        # Yaw (for logging only)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.drone_yaw = np.arctan2(siny, cosy)
        # Full rotation matrix from quaternion (body → world)
        # Accounts for roll, pitch, AND yaw — critical during forward flight
        # when the drone tilts nose-down.
        xx = q.x * q.x
        yy = q.y * q.y
        zz = q.z * q.z
        xy = q.x * q.y
        xz = q.x * q.z
        yz = q.y * q.z
        wx = q.w * q.x
        wy = q.w * q.y
        wz = q.w * q.z
        self._R_body_to_world = np.array([
            [1 - 2*(yy+zz),  2*(xy-wz),      2*(xz+wy)],
            [2*(xy+wz),      1 - 2*(xx+zz),   2*(yz-wx)],
            [2*(xz-wy),      2*(yz+wx),       1 - 2*(xx+yy)],
        ])
        self._have_pose = True

    # ------------------------------------------------------------------
    def pointcloud_callback(self, msg):
        """Outer callback — catches any exception so the node never dies."""
        try:
            self._process_cloud(msg)
        except Exception as e:
            tb = traceback.format_exc()
            self.get_logger().error(
                f'pointcloud error: {e}\n{tb}',
                throttle_duration_sec=10.0)
            self.flog.error(f'pointcloud error: {e}\n{tb}')

    # ------------------------------------------------------------------
    def _process_cloud(self, msg):
        # Wait until we have a valid MAVROS pose
        if not self._have_pose:
            return

        # ---- Read structured point cloud ----
        # read_points → structured array (named fields: 'x','y','z')
        # read_points_numpy → unstructured 2D array (column indices only)
        pts_raw = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if len(pts_raw) == 0:
            return

        px = np.asarray(pts_raw['x'], dtype=np.float64)
        py = np.asarray(pts_raw['y'], dtype=np.float64)
        pz = np.asarray(pts_raw['z'], dtype=np.float64)

        # Filter out inf values (skip_nans doesn't catch these)
        finite = np.isfinite(px) & np.isfinite(py) & np.isfinite(pz)
        px, py, pz = px[finite], py[finite], pz[finite]
        if len(px) == 0:
            return

        # ---- Auto-detect frame convention on first valid cloud ----
        if self._frame is None:
            ax = float(np.max(np.abs(px)))
            ay = float(np.max(np.abs(py)))
            az = float(np.max(np.abs(pz)))
            if az > ax and az > ay:
                self._frame = 'optical'
            else:
                self._frame = 'sensor_link'
            frame_msg = (
                f'Frame auto-detect: {self._frame.upper()}  '
                f'max|x|={ax:.2f}  max|y|={ay:.2f}  max|z|={az:.2f}')
            self.get_logger().info(frame_msg)
            self.flog.info(frame_msg)

        # ---- Debug: first 3 frames ----
        if self._debug_count < 3:
            self._debug_count += 1
            cloud_msg = (
                f'[CLOUD #{self._debug_count}] {len(px)} pts  '
                f'x=[{px.min():.2f},{px.max():.2f}]  '
                f'y=[{py.min():.2f},{py.max():.2f}]  '
                f'z=[{pz.min():.2f},{pz.max():.2f}]  '
                f'drone=({self.drone_x:.2f},{self.drone_y:.2f},'
                f'{self.drone_z:.2f})  yaw={self.drone_yaw:.3f}  '
                f'frame={self._frame}')
            self.get_logger().info(cloud_msg)
            self.flog.info(cloud_msg)

        # 1. Range filter — ignore very close + beyond max_range
        dists = np.sqrt(px * px + py * py + pz * pz)
        mask = (dists > 0.3) & (dists < self.max_range)
        px, py, pz = px[mask], py[mask], pz[mask]
        if len(px) == 0:
            # Decay only unconfirmed cells (below threshold);
            # confirmed obstacles are permanent.
            unconfirmed = self.hit_count < self.min_hits
            self.hit_count[unconfirmed] = np.maximum(
                self.hit_count[unconfirmed] - np.int16(self.hit_decay),
                np.int16(0))
            self._publish_grid()
            return

        # 2. Convert to "sensor-link" axes: fwd / left / up
        if self._frame == 'optical':
            # optical: x=right, y=down, z=depth
            s_fwd = pz
            s_left = -px
            s_up = -py
        else:
            # sensor-link: x=forward, y=left, z=up
            s_fwd = px
            s_left = py
            s_up = pz

        # 3. Undo fixed camera pitch → get point in body frame
        #    Camera is pitched cam_pitch rad DOWNWARD from body-forward.
        cp = np.cos(self.cam_pitch)
        sp = np.sin(self.cam_pitch)
        body_fwd = cp * s_fwd + sp * s_up
        body_left = s_left
        body_up = -sp * s_fwd + cp * s_up

        # 4. Body frame → World frame using FULL quaternion rotation
        #    This accounts for drone roll AND pitch during flight
        #    (not just yaw). Critical: when the drone pitches forward
        #    to fly, ignoring that tilt would bias world_z upward,
        #    causing ground points to pass the height filter.
        R = self._R_body_to_world
        world_pts_x = R[0, 0] * body_fwd + R[0, 1] * body_left + R[0, 2] * body_up + self.drone_x
        world_pts_y = R[1, 0] * body_fwd + R[1, 1] * body_left + R[1, 2] * body_up + self.drone_y
        world_z     = R[2, 0] * body_fwd + R[2, 1] * body_left + R[2, 2] * body_up + self.cam_z_offset + self.drone_z

        # 5. Height filter
        keep = ((world_z > self.min_obstacle_height) & (world_z < 4.0))

        n_pass = int(keep.sum())
        hf_msg = (
            f'  height filter: {n_pass}/{len(keep)} kept  '
            f'world_z=[{world_z.min():.2f},{world_z.max():.2f}]')
        self.flog.info(hf_msg)
        if n_pass > 0:
            self.flog.info(
                f'  drone=({self.drone_x:.2f},{self.drone_y:.2f},'
                f'{self.drone_z:.2f})  yaw={self.drone_yaw:.3f}')

        world_pts_x = world_pts_x[keep]
        world_pts_y = world_pts_y[keep]
        world_z_kept = world_z[keep]

        # --- Decay only UNCONFIRMED cells every frame ---
        #     Cells below min_hits decay so transient noise fades.
        #     Cells that have reached min_hits are permanent obstacles.
        unconfirmed = self.hit_count < self.min_hits
        self.hit_count[unconfirmed] = np.maximum(
            self.hit_count[unconfirmed] - np.int16(self.hit_decay),
            np.int16(0))

        if len(world_pts_x) == 0:
            self._publish_grid()
            return

        # Use world XY directly (already in world frame)
        world_x = world_pts_x
        world_y = world_pts_y

        self.flog.info(
            f'  world XY: x=[{world_x.min():.2f},{world_x.max():.2f}]  '
            f'y=[{world_y.min():.2f},{world_y.max():.2f}]  '
            f'{len(world_x)} obstacle pts')

        # 6. Rasterise — per-frame BINARY detection + increment
        #    Each cell gets at most +hit_increment per frame,
        #    regardless of how many points fall in it.
        col = np.round(
            (world_x - self.grid_origin_x) / self.resolution
        ).astype(np.int32)
        row = np.round(
            (world_y - self.grid_origin_y) / self.resolution
        ).astype(np.int32)
        valid = ((col >= 0) & (col < self.grid_w)
                 & (row >= 0) & (row < self.grid_h))
        r_v, c_v = row[valid], col[valid]

        # Binary mask: which cells were detected THIS frame?
        detected_this_frame = np.zeros(
            (self.grid_h, self.grid_w), dtype=bool)
        detected_this_frame[r_v, c_v] = True

        # Increment only detected cells (cap to avoid overflow)
        self.hit_count[detected_this_frame] += np.int16(
            self.hit_increment)
        np.clip(self.hit_count, 0, self.min_hits * 3,
                out=self.hit_count)

        self._publish_grid()

    # ------------------------------------------------------------------
    def _publish_grid(self):
        """Build occupied grid from hit-counts, inflate, and publish."""
        # Only cells with enough independent confirmations are obstacles
        raw_grid = np.where(
            self.hit_count >= self.min_hits,
            np.int8(100), np.int8(0))

        inflate_cells = int(round(self.inflate_radius / self.resolution))
        if inflate_cells > 0:
            occupied = (raw_grid == 100)
            inflated = occupied.copy()
            for _ in range(inflate_cells):
                pad = np.pad(inflated, 1, mode='constant',
                             constant_values=False)
                inflated = (
                    pad[1:-1, 1:-1]
                    | pad[:-2, 1:-1] | pad[2:, 1:-1]
                    | pad[1:-1, :-2] | pad[1:-1, 2:]
                    | pad[:-2, :-2] | pad[:-2, 2:]
                    | pad[2:, :-2] | pad[2:, 2:])
            grid_out = np.zeros_like(raw_grid)
            grid_out[inflated] = 100
        else:
            grid_out = raw_grid

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.grid_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.grid_origin_x
        msg.info.origin.position.y = self.grid_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_out.flatten().tolist()
        self.map_pub.publish(msg)

        # Periodic status log (every 30 s)
        now = self.get_clock().now()
        if (self._last_log_time is None
                or (now - self._last_log_time).nanoseconds
                >= 30_000_000_000):
            raw_n = int(np.count_nonzero(raw_grid == 100))
            inf_n = int(np.count_nonzero(grid_out == 100))
            total = self.grid_h * self.grid_w
            grid_msg = (
                f'Grid: {raw_n} raw occupied, {inf_n} inflated '
                f'({100.0 * inf_n / total:.1f}% of {total})')
            self.get_logger().info(grid_msg)
            self.flog.info(grid_msg)
            self._last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
