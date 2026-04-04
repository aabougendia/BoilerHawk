#!/usr/bin/env python3
"""Human detector node — YOLOv8n inference on depth camera RGB stream.

Copyright 2026 BoilerHawk — MIT License.

Subscribes to the RGB image from the depth camera, runs YOLOv8n at a
throttled rate, and publishes detected-person poses in the world frame
by combining the bounding-box centre with the matching depth pixel.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False


class HumanDetectorNode(Node):
    """Run YOLOv8n on camera images and publish person detections."""

    def __init__(self) -> None:
        super().__init__('human_detector_node')

        # Parameters
        self.declare_parameter('image_topic', '/depth_camera/image')
        self.declare_parameter('depth_topic', '/depth_camera/depth_image')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('detection_topic', '/detection/pose')
        self.declare_parameter('detection_image_topic', '/detection/image')
        self.declare_parameter('detection_info_topic', '/detection/info')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('detect_rate', 5.0)          # Hz
        self.declare_parameter('consecutive_required', 3)    # frames
        self.declare_parameter('cam_pitch', 0.2)             # radians
        self.declare_parameter('cam_z_offset', 0.05)
        self.declare_parameter('cam_hfov', 1.047)            # 60 degrees

        self._image_topic = str(
            self.get_parameter('image_topic').value)
        self._depth_topic = str(
            self.get_parameter('depth_topic').value)
        self._pose_topic = str(
            self.get_parameter('pose_topic').value)
        self._detection_topic = str(
            self.get_parameter('detection_topic').value)
        self._detection_image_topic = str(
            self.get_parameter('detection_image_topic').value)
        self._detection_info_topic = str(
            self.get_parameter('detection_info_topic').value)
        self._model_path = str(
            self.get_parameter('model_path').value)
        self._conf_thresh = float(
            self.get_parameter('confidence_threshold').value)
        self._detect_rate = float(
            self.get_parameter('detect_rate').value)
        self._consec_required = int(
            self.get_parameter('consecutive_required').value)
        self._cam_pitch = float(
            self.get_parameter('cam_pitch').value)
        self._cam_z_offset = float(
            self.get_parameter('cam_z_offset').value)
        self._cam_hfov = float(
            self.get_parameter('cam_hfov').value)

        # State
        self._bridge = CvBridge()
        self._latest_rgb = None
        self._latest_depth = None
        self._drone_x = 0.0
        self._drone_y = 0.0
        self._drone_z = 0.0
        self._R_body = np.eye(3)
        self._have_pose = False
        self._consec_count = 0
        self._detection_published = False

        # Load YOLO model
        if not _YOLO_AVAILABLE:
            self.get_logger().error(
                'ultralytics not installed — '
                'run: pip install ultralytics')
            return

        self.get_logger().info(
            f'Loading YOLO model: {self._model_path}')
        self._model = YOLO(self._model_path)
        self.get_logger().info('YOLO model loaded')

        # Subscriptions
        self.create_subscription(
            Image, self._image_topic, self._rgb_cb, 10)
        self.create_subscription(
            Image, self._depth_topic, self._depth_cb, 10)

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(
            PoseStamped, self._pose_topic, self._pose_cb, mavros_qos)

        # Publishers
        self._det_pub = self.create_publisher(
            PoseStamped, self._detection_topic, 10)
        self._det_img_pub = self.create_publisher(
            Image, self._detection_image_topic, 10)
        self._det_info_pub = self.create_publisher(
            String, self._detection_info_topic, 10)

        # Timer — throttled inference
        period = 1.0 / max(self._detect_rate, 0.1)
        self._timer = self.create_timer(period, self._detect_tick)

        self.get_logger().info(
            f'Human detector running at {self._detect_rate} Hz, '
            f'conf>={self._conf_thresh}, '
            f'require {self._consec_required} consecutive')

    # ------------------------------------------------------------------ #
    #  Callbacks
    # ------------------------------------------------------------------ #

    def _rgb_cb(self, msg: Image) -> None:
        self._latest_rgb = msg

    def _depth_cb(self, msg: Image) -> None:
        self._latest_depth = msg

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._drone_x = msg.pose.position.x
        self._drone_y = msg.pose.position.y
        self._drone_z = msg.pose.position.z
        q = msg.pose.orientation
        xx = q.x * q.x; yy = q.y * q.y; zz = q.z * q.z
        xy = q.x * q.y; xz = q.x * q.z; yz = q.y * q.z
        wx = q.w * q.x; wy = q.w * q.y; wz = q.w * q.z
        self._R_body = np.array([
            [1 - 2*(yy+zz), 2*(xy-wz),     2*(xz+wy)],
            [2*(xy+wz),     1 - 2*(xx+zz),  2*(yz-wx)],
            [2*(xz-wy),     2*(yz+wx),      1 - 2*(xx+yy)],
        ])
        self._have_pose = True

    # ------------------------------------------------------------------ #
    #  Detection tick
    # ------------------------------------------------------------------ #

    def _detect_tick(self) -> None:
        if self._latest_rgb is None or not self._have_pose:
            return

        # Convert ROS Image to OpenCV BGR
        try:
            cv_img = self._bridge.imgmsg_to_cv2(
                self._latest_rgb, desired_encoding='bgr8')
        except Exception:
            return

        # Run YOLOv8 inference
        results = self._model(cv_img, verbose=False, conf=self._conf_thresh)

        # Filter for 'person' class (COCO class 0)
        best_person = None
        best_conf = 0.0
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > best_conf:
                    best_conf = conf
                    best_person = box

        if best_person is not None:
            self._consec_count += 1

            # Publish annotated image every detection (for RViz / debug)
            self._publish_annotated_image(cv_img, results)

            info_msg = String()
            info_msg.data = (
                f'Person detected (conf={best_conf:.2f}, '
                f'streak={self._consec_count}/{self._consec_required})')
            self._det_info_pub.publish(info_msg)

            if (self._consec_count >= self._consec_required
                    and not self._detection_published):
                # Compute world-frame position of the detected person
                pose = self._bbox_to_world_pose(best_person)
                if pose is not None:
                    self._det_pub.publish(pose)
                    self._detection_published = True
                    p = pose.pose.position
                    self.get_logger().info(
                        f'HUMAN DETECTED at ({p.x:.1f}, {p.y:.1f}) '
                        f'conf={best_conf:.2f}')
        else:
            self._consec_count = 0

    # ------------------------------------------------------------------ #
    #  Pixel → world coordinate
    # ------------------------------------------------------------------ #

    def _bbox_to_world_pose(self, box) -> PoseStamped:
        """Convert bounding box centre + depth to world PoseStamped."""
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        # Get depth at detection centre
        depth_m = self._get_depth_at(cx, cy)
        if depth_m is None or depth_m < 0.3 or depth_m > 20.0:
            return None

        # Pixel → camera-frame ray (pinhole model)
        if self._latest_rgb is None:
            return None
        img_w = self._latest_rgb.width
        img_h = self._latest_rgb.height
        fx = (img_w / 2.0) / math.tan(self._cam_hfov / 2.0)
        fy = fx  # square pixels

        # Camera optical frame: x=right, y=down, z=forward
        cam_z = depth_m
        cam_x = (cx - img_w / 2.0) * depth_m / fx
        cam_y = (cy - img_h / 2.0) * depth_m / fy

        # Optical → sensor-link (fwd/left/up)
        s_fwd = cam_z
        s_left = -cam_x
        s_up = -cam_y

        # Undo camera pitch
        cp = math.cos(self._cam_pitch)
        sp = math.sin(self._cam_pitch)
        body_fwd = cp * s_fwd + sp * s_up
        body_left = s_left
        body_up = -sp * s_fwd + cp * s_up

        # Body → world
        R = self._R_body
        wx = (R[0, 0] * body_fwd + R[0, 1] * body_left
              + R[0, 2] * body_up + self._drone_x)
        wy = (R[1, 0] * body_fwd + R[1, 1] * body_left
              + R[1, 2] * body_up + self._drone_y)

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.position.z = 0.0  # ground level
        pose.pose.orientation.w = 1.0
        return pose

    def _get_depth_at(self, cx: int, cy: int):
        """Read depth value at pixel (cx, cy) from latest depth image."""
        if self._latest_depth is None:
            return None
        try:
            depth_img = self._bridge.imgmsg_to_cv2(
                self._latest_depth, desired_encoding='passthrough')
        except Exception:
            return None

        h, w = depth_img.shape[:2]
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return None

        # Sample a small patch for robustness
        r = 3
        y0 = max(0, cy - r)
        y1 = min(h, cy + r + 1)
        x0 = max(0, cx - r)
        x1 = min(w, cx + r + 1)
        patch = depth_img[y0:y1, x0:x1].astype(np.float64)
        valid = patch[(patch > 0.3) & (patch < 20.0) & np.isfinite(patch)]
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    # ------------------------------------------------------------------ #
    #  Annotated image publishing
    # ------------------------------------------------------------------ #

    def _publish_annotated_image(self, cv_img, results) -> None:
        """Publish image with YOLO bounding boxes drawn."""
        try:
            import cv2
            annotated = cv_img.copy()
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    if cls_id != 0:
                        continue
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = float(box.conf[0])
                    cv2.rectangle(annotated, (x1, y1), (x2, y2),
                                  (0, 255, 0), 2)
                    cv2.putText(annotated, f'person {conf:.2f}',
                                (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1)
            msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self._det_img_pub.publish(msg)
        except Exception:
            pass

    def reset_detection(self) -> None:
        """Allow detecting again (called after mission reacts)."""
        self._detection_published = False
        self._consec_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
