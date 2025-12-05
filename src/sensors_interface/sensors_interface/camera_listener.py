import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraListener(Node):
    def __init__(self):
        super().__init__('camera_listener')
        self.bridge = CvBridge()

        # Subscribe to RGB image
        self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)

        # Subscribe to depth image
        self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, 10)

        # Subscribe to point cloud
        self.create_subscription(
            PointCloud2, '/camera/points', self.points_callback, 10)

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("RGB Camera", img)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        norm_depth = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow("Depth Map", np.uint8(norm_depth))
        cv2.waitKey(1)

    def points_callback(self, msg):
        self.get_logger().info(f"Received point cloud: {msg.width * msg.height} points")

def main(args=None):
    rclpy.init(args=args)
    node = CameraListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
