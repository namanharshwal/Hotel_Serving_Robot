#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - Live Camera Feed Viewer for Manual Teleop & Perception
=============================================================================
Description:
  High-performance PyQt5 video viewer for the Logitech C270 HD camera.
  Designed specifically for physical manual teleoperation:
    - Real-time 30 FPS video streaming from /camera/image_raw
    - Visual heading corridor and center crosshair for precise robot positioning
    - Real-time FPS, resolution, and ultrasonic obstacle proximity alerts
    - Beautiful dark-themed aesthetic with automatic positioning next to teleop

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
=============================================================================
"""

import sys
import os
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QFrame, QSizePolicy
)
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QFont
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject


class CameraSignalBridge(QObject):
    frame_ready = pyqtSignal(np.ndarray)
    sonar_alert = pyqtSignal(bool, float)


class CameraViewerNode(Node):
    def __init__(self, bridge: CameraSignalBridge):
        super().__init__('camera_viewer_node')
        self.bridge = bridge

        self.sonar_front_dist = 4.5
        self.sonar_critical = False

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.sonar_front_sub = self.create_subscription(
            Range, '/sonar/front', self.sonar_callback, 10
        )
        self.stop_sub = self.create_subscription(
            Bool, '/nammy/sonar_safety_stop', self.safety_callback, 10
        )

        self.get_logger().info("Camera Viewer Node initialized: streaming /camera/image_raw")

    def image_callback(self, msg: Image):
        try:
            # Handle standard BGR8 or RGB8 encodings directly via numpy
            if msg.encoding in ('bgr8', 'rgb8'):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                if msg.encoding == 'bgr8':
                    # Convert BGR to RGB for Qt display
                    rgb_arr = arr[:, :, ::-1].copy()
                else:
                    rgb_arr = arr.copy()
                self.bridge.frame_ready.emit(rgb_arr)
        except Exception as e:
            self.get_logger().warn(f"Frame decode error: {e}")

    def sonar_callback(self, msg: Range):
        self.sonar_front_dist = msg.range
        self.bridge.sonar_alert.emit(self.sonar_critical, self.sonar_front_dist)

    def safety_callback(self, msg: Bool):
        self.sonar_critical = msg.data
        self.bridge.sonar_alert.emit(self.sonar_critical, self.sonar_front_dist)


class CameraViewerWindow(QMainWindow):
    def __init__(self, bridge: CameraSignalBridge):
        super().__init__()
        self.bridge = bridge

        self.setWindowTitle("Nammy Robot - Live Vision Feed (Logitech C270)")
        self.setGeometry(630, 40, 620, 520)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0f172a;
            }
            QLabel {
                color: #e2e8f0;
                font-family: 'DejaVu Sans', 'Monospace';
            }
        """)

        # Main widget & layout
        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(6)

        # Header bar
        header_layout = QHBoxLayout()
        self.title_label = QLabel("● LIVE VISION STREAM")
        self.title_label.setFont(QFont("DejaVu Sans", 11, QFont.Bold))
        self.title_label.setStyleSheet("color: #00ff88;")

        self.fps_label = QLabel("FPS: -- | 640x480")
        self.fps_label.setFont(QFont("Monospace", 9))
        self.fps_label.setStyleSheet("color: #94a3b8;")

        header_layout.addWidget(self.title_label)
        header_layout.addStretch()
        header_layout.addWidget(self.fps_label)
        layout.addLayout(header_layout)

        # Video frame display label
        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.video_label.setStyleSheet("""
            background-color: #020617;
            border: 2px solid #1e293b;
            border-radius: 8px;
        """)
        self.video_label.setText("Waiting for /camera/image_raw stream...")
        layout.addWidget(self.video_label)

        # Bottom Teleop HUD Status Bar
        footer_layout = QHBoxLayout()
        self.mode_label = QLabel("MODE: PHYSICAL MANUAL TELEOP")
        self.mode_label.setFont(QFont("DejaVu Sans", 9, QFont.Bold))
        self.mode_label.setStyleSheet("color: #38bdf8;")

        self.sonar_label = QLabel("SONAR FRONT: CLEAR (>4.5m)")
        self.sonar_label.setFont(QFont("Monospace", 9, QFont.Bold))
        self.sonar_label.setStyleSheet("color: #22c55e;")

        footer_layout.addWidget(self.mode_label)
        footer_layout.addStretch()
        footer_layout.addWidget(self.sonar_label)
        layout.addLayout(footer_layout)

        # Frame counting
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0

        # Connect signals
        self.bridge.frame_ready.connect(self.update_frame)
        self.bridge.sonar_alert.connect(self.update_sonar)

    def update_frame(self, rgb_arr: np.ndarray):
        h, w, ch = rgb_arr.shape
        bytes_per_line = ch * w

        qimg = QImage(rgb_arr.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        # Draw HUD overlays on top of the frame
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        # 1. Center Crosshairs
        cx, cy = w // 2, h // 2
        pen_green = QPen(QColor(0, 255, 136, 180), 2, Qt.SolidLine)
        painter.setPen(pen_green)
        painter.drawLine(cx - 25, cy, cx + 25, cy)
        painter.drawLine(cx, cy - 25, cx, cy + 25)
        painter.drawEllipse(cx - 8, cy - 8, 16, 16)

        # 2. Driving Corridor Trajectory Lines (guidance for narrow doorways / obstacles)
        corridor_pen = QPen(QColor(56, 189, 248, 140), 2, Qt.DashLine)
        painter.setPen(corridor_pen)
        # Left guidance rail
        painter.drawLine(int(w * 0.25), h, int(w * 0.40), int(h * 0.60))
        # Right guidance rail
        painter.drawLine(int(w * 0.75), h, int(w * 0.60), int(h * 0.60))
        # Ground distance hash lines
        painter.drawLine(int(w * 0.30), int(h * 0.85), int(w * 0.70), int(h * 0.85))
        painter.drawLine(int(w * 0.35), int(h * 0.70), int(w * 0.65), int(h * 0.70))

        # 3. Label overlay
        painter.setFont(QFont("Monospace", 9, QFont.Bold))
        painter.setPen(QColor(255, 255, 255, 200))
        painter.drawText(15, 25, "LOGITECH C270 HD")

        painter.end()

        # Scale to label size keeping aspect ratio
        scaled = pixmap.scaled(
            self.video_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.video_label.setPixmap(scaled)

        # Calculate FPS
        self.frame_count += 1
        now = time.time()
        elapsed = now - self.last_fps_time
        if elapsed >= 1.0:
            self.current_fps = self.frame_count / elapsed
            self.frame_count = 0
            self.last_fps_time = now
            self.fps_label.setText(f"FPS: {self.current_fps:.1f} | {w}x{h}")

    def update_sonar(self, critical: bool, dist: float):
        if critical or dist < 0.15:
            self.sonar_label.setText(f"SONAR FRONT: CRITICAL STOP ({dist*100:.0f} cm)")
            self.sonar_label.setStyleSheet("color: #ef4444; font-weight: bold;")
        elif dist < 0.50:
            self.sonar_label.setText(f"SONAR FRONT: OBSTACLE NEAR ({dist*100:.0f} cm)")
            self.sonar_label.setStyleSheet("color: #eab308; font-weight: bold;")
        else:
            self.sonar_label.setText(f"SONAR FRONT: CLEAR ({dist:4.2f}m)")
            self.sonar_label.setStyleSheet("color: #22c55e;")


def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)

    bridge = CameraSignalBridge()
    window = CameraViewerWindow(bridge)
    node = CameraViewerNode(bridge)

    # ROS 2 executor thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    window.show()

    ret = app.exec_()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
