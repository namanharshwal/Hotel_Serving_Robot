#!/usr/bin/env python3
"""
Logitech C270 HD Webcam ROS 2 Publisher Node for Nammy Robot.
Streams raw video frames and CameraInfo metadata for perception and visualization.
Author: Naman Harshwal <namanharshwal@gmail.com>
"""

import os
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class C270CameraNode(Node):
    def __init__(self):
        super().__init__('c270_camera_node')

        # Declare parameters
        self.declare_parameter('device_path', '/dev/video0')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('framerate', 30.0)
        self.declare_parameter('camera_name', 'c270_logitech_camera')
        self.declare_parameter('auto_reconnect', True)

        self.device_path = self.get_parameter('device_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().double_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.auto_reconnect = self.get_parameter('auto_reconnect').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.cap = None

        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Pre-compute CameraInfo for Logitech C270 (FOV ~60 deg diagonal)
        self.camera_info = self.build_camera_info()

        # Connect to camera
        self.connect_camera()

        # Timer loop at desired framerate
        timer_period = 1.0 / max(1.0, self.framerate)
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

        self.get_logger().info(f"Logitech C270 Camera Node started on {self.device_path} ({self.width}x{self.height} @ {self.framerate:.1f} FPS)")

    def build_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Nominal C270 pinhole intrinsic matrix for 640x480 (f ~ 800px)
        fx = 800.0 * (self.width / 640.0)
        fy = 800.0 * (self.height / 480.0)
        cx = self.width / 2.0
        cy = self.height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def connect_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        if not os.path.exists(self.device_path):
            self.get_logger().warn(f"Camera device {self.device_path} not found. Will retry...")
            return False

        try:
            self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.framerate)

            if self.cap.isOpened():
                self.get_logger().info(f"[SUCCESS] Opened Logitech C270 on {self.device_path}")
                return True
            else:
                self.get_logger().warn(f"Failed to open VideoCapture on {self.device_path}")
                self.cap = None
                return False
        except Exception as e:
            self.get_logger().error(f"Error opening camera: {e}")
            self.cap = None
            return False

    def capture_and_publish(self):
        if self.cap is None or not self.cap.isOpened():
            if self.auto_reconnect:
                self.connect_camera()
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Empty frame captured from camera. Reconnecting...")
            self.cap.release()
            self.cap = None
            return

        now = self.get_clock().now()
        timestamp = now.to_msg()

        # Convert to ROS 2 Image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = timestamp
        img_msg.header.frame_id = self.frame_id

        # Update CameraInfo timestamp
        self.camera_info.header.stamp = timestamp

        # Publish both
        self.image_pub.publish(img_msg)
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = C270CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
