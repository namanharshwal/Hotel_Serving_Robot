#!/usr/bin/env python3
"""
10.1-Inch (1280x800) Capacitive Touch HMI Dashboard for Nammy Robot.
Designed specifically for Waveshare/ThinkRobotics WVSH0067 IPS HDMI display.
Provides touch teleoperation, 24V LiFePO4 battery monitoring, and instant E-STOP.
"""

import sys
import math
import time
import threading
import tkinter as tk
from tkinter import ttk, font

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

class DashboardROSNode(Node):
    def __init__(self, ui_bridge):
        super().__init__('nammy_touch_dashboard')
        self.ui_bridge = ui_bridge

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.batt_sub = self.create_subscription(BatteryState, 'nammy/battery_state', self.batt_cb, 10)

    def odom_cb(self, msg: Odometry):
        self.ui_bridge.update_odom(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        )

    def batt_cb(self, msg: BatteryState):
        self.ui_bridge.update_battery(msg.voltage, msg.percentage)

    def send_twist(self, linear_x, angular_z):
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

class TouchHMIApplication:
    def __init__(self, root):
        self.root = root
        self.root.title("NAMMY ROBOT OS - 10.1\" TOUCH HMI")
        self.root.geometry("1280x800")
        self.root.configure(bg="#0f172a")  # Deep slate navy background

        # State
        self.linear_v = 0.0
        self.angular_w = 0.0
        self.target_speed = 0.35  # m/s
        self.target_turn = 0.80   # rad/s
        self.estop_active = False
        self.current_mode = "MANUAL TELEOP"
        
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.actual_v = 0.0
        self.actual_w = 0.0
        self.batt_voltage = 26.4
        self.batt_pct = 0.85

        # Initialize ROS 2 Node in separate thread
        self.ros_node = DashboardROSNode(self)
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

        # Build GUI Layout
        self.build_ui()

        # Periodic GUI update timer (30 Hz)
        self.root.after(33, self.periodic_ui_update)

    def spin_ros(self):
        rclpy.spin(self.ros_node)

    def build_ui(self):
        # 1. Top Header Bar
        header = tk.Frame(self.root, bg="#1e293b", height=70)
        header.pack(fill=tk.X, side=tk.TOP)

        title_lbl = tk.Label(header, text="NAMMY ROBOT PLATFORM", font=("Arial", 22, "bold"), fg="#38bdf8", bg="#1e293b")
        title_lbl.pack(side=tk.LEFT, padx=25, pady=15)

        self.heartbeat_lbl = tk.Label(header, text="● ESP32-S3 ONLINE", font=("Arial", 14, "bold"), fg="#22c55e", bg="#1e293b")
        self.heartbeat_lbl.pack(side=tk.LEFT, padx=20)

        self.time_lbl = tk.Label(header, text="", font=("Arial", 16), fg="#94a3b8", bg="#1e293b")
        self.time_lbl.pack(side=tk.RIGHT, padx=25)

        # 2. Main Content Split into 3 Columns
        content = tk.Frame(self.root, bg="#0f172a")
        content.pack(fill=tk.BOTH, expand=True, padx=20, pady=15)

        # Column 1: Power & Odometry (Width 360)
        col1 = tk.Frame(content, bg="#1e293b", width=360, relief=tk.RIDGE, bd=2)
        col1.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.build_col1(col1)

        # Column 2: Center Touch Driving D-Pad (Width 440)
        col2 = tk.Frame(content, bg="#1e293b", width=440, relief=tk.RIDGE, bd=2)
        col2.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.build_col2(col2)

        # Column 3: E-Stop & Diagnostics (Width 360)
        col3 = tk.Frame(content, bg="#1e293b", width=360, relief=tk.RIDGE, bd=2)
        col3.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.build_col3(col3)

    def build_col1(self, parent):
        tk.Label(parent, text="POWER & TELEMETRY", font=("Arial", 16, "bold"), fg="#f8fafc", bg="#1e293b").pack(pady=12)

        # Battery Gauge Card
        batt_card = tk.Frame(parent, bg="#334155", padx=15, pady=12)
        batt_card.pack(fill=tk.X, padx=15, pady=5)

        tk.Label(batt_card, text="24V 15Ah LiFePO4 (380Wh)", font=("Arial", 12, "bold"), fg="#94a3b8", bg="#334155").pack(anchor=tk.W)
        self.batt_v_lbl = tk.Label(batt_card, text="26.40 V", font=("Arial", 32, "bold"), fg="#22c55e", bg="#334155")
        self.batt_v_lbl.pack(anchor=tk.W, pady=2)

        # Battery Canvas Bar
        self.batt_canvas = tk.Canvas(batt_card, height=26, bg="#1e293b", highlightthickness=0)
        self.batt_canvas.pack(fill=tk.X, pady=8)
        self.batt_bar = self.batt_canvas.create_rectangle(0, 0, 0, 26, fill="#22c55e", width=0)
        self.batt_pct_lbl = tk.Label(batt_card, text="85% State of Charge (Good)", font=("Arial", 12), fg="#f8fafc", bg="#334155")
        self.batt_pct_lbl.pack(anchor=tk.W)

        # Motion Telemetry Card
        tele_card = tk.Frame(parent, bg="#334155", padx=15, pady=12)
        tele_card.pack(fill=tk.X, padx=15, pady=15)

        tk.Label(tele_card, text="REAL-TIME VELOCITY", font=("Arial", 12, "bold"), fg="#94a3b8", bg="#334155").pack(anchor=tk.W)
        self.vel_lbl = tk.Label(tele_card, text="v: +0.00 m/s\nω: +0.00 rad/s", font=("Courier", 16, "bold"), fg="#38bdf8", bg="#334155", justify=tk.LEFT)
        self.vel_lbl.pack(anchor=tk.W, pady=6)

        # Odometry Pose
        tk.Label(tele_card, text="ODOMETRY POSE", font=("Arial", 12, "bold"), fg="#94a3b8", bg="#334155").pack(anchor=tk.W, pady=(8, 0))
        self.pose_lbl = tk.Label(tele_card, text="X: 0.00 m\nY: 0.00 m", font=("Courier", 14), fg="#e2e8f0", bg="#334155", justify=tk.LEFT)
        self.pose_lbl.pack(anchor=tk.W, pady=4)

    def build_col2(self, parent):
        tk.Label(parent, text="TOUCH TELEOPERATION", font=("Arial", 16, "bold"), fg="#f8fafc", bg="#1e293b").pack(pady=12)

        # Mode Buttons
        mode_frame = tk.Frame(parent, bg="#1e293b")
        mode_frame.pack(fill=tk.X, padx=15, pady=5)
        
        self.btn_manual = tk.Button(mode_frame, text="MANUAL", font=("Arial", 12, "bold"), bg="#0284c7", fg="white", height=2, command=lambda: self.set_mode("MANUAL TELEOP"))
        self.btn_manual.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        self.btn_nav = tk.Button(mode_frame, text="NAV2 AUTO", font=("Arial", 12, "bold"), bg="#475569", fg="white", height=2, command=lambda: self.set_mode("AUTONOMOUS NAV2"))
        self.btn_nav.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        # Big Touch Driving D-Pad (Touch Hit Targets > 75px)
        dpad = tk.Frame(parent, bg="#1e293b")
        dpad.pack(pady=20)

        # Forward
        btn_fwd = tk.Button(dpad, text="▲\nFORWARD", font=("Arial", 14, "bold"), bg="#334155", fg="white", width=12, height=3,
                            command=lambda: self.touch_drive(self.target_speed, 0.0))
        btn_fwd.grid(row=0, column=1, padx=6, pady=6)

        # Turn Left
        btn_left = tk.Button(dpad, text="◄\nLEFT", font=("Arial", 14, "bold"), bg="#334155", fg="white", width=10, height=3,
                             command=lambda: self.touch_drive(0.0, self.target_turn))
        btn_left.grid(row=1, column=0, padx=6, pady=6)

        # Stop
        btn_stop = tk.Button(dpad, text="■\nSTOP", font=("Arial", 14, "bold"), bg="#ea580c", fg="white", width=10, height=3,
                             command=lambda: self.touch_drive(0.0, 0.0))
        btn_stop.grid(row=1, column=1, padx=6, pady=6)

        # Turn Right
        btn_right = tk.Button(dpad, text="►\nRIGHT", font=("Arial", 14, "bold"), bg="#334155", fg="white", width=10, height=3,
                              command=lambda: self.touch_drive(0.0, -self.target_turn))
        btn_right.grid(row=1, column=2, padx=6, pady=6)

        # Backward
        btn_back = tk.Button(dpad, text="▼\nREVERSE", font=("Arial", 14, "bold"), bg="#334155", fg="white", width=12, height=3,
                             command=lambda: self.touch_drive(-self.target_speed, 0.0))
        btn_back.grid(row=2, column=1, padx=6, pady=6)

        # Speed Stepper
        speed_box = tk.Frame(parent, bg="#1e293b")
        speed_box.pack(pady=10)
        tk.Button(speed_box, text="- Speed", font=("Arial", 12, "bold"), bg="#475569", fg="white", padx=15, pady=8, command=self.dec_speed).pack(side=tk.LEFT, padx=10)
        self.speed_lbl = tk.Label(speed_box, text=f"Max: {self.target_speed:.2f} m/s", font=("Arial", 14, "bold"), fg="#38bdf8", bg="#1e293b")
        self.speed_lbl.pack(side=tk.LEFT, padx=10)
        tk.Button(speed_box, text="+ Speed", font=("Arial", 12, "bold"), bg="#475569", fg="white", padx=15, pady=8, command=self.inc_speed).pack(side=tk.LEFT, padx=10)

    def build_col3(self, parent):
        tk.Label(parent, text="SAFETY & HARDWARE", font=("Arial", 16, "bold"), fg="#f8fafc", bg="#1e293b").pack(pady=12)

        # Huge High-Contrast Red E-STOP Button (Touch Target 150px)
        self.btn_estop = tk.Button(parent, text="EMERGENCY\nSTOP", font=("Arial", 22, "bold"), bg="#dc2626", fg="white",
                                   activebackground="#991b1b", height=4, relief=tk.RAISED, bd=6, command=self.toggle_estop)
        self.btn_estop.pack(fill=tk.X, padx=20, pady=10)

        # Hardware Status Matrix
        hw_card = tk.Frame(parent, bg="#334155", padx=15, pady=15)
        hw_card.pack(fill=tk.BOTH, expand=True, padx=20, pady=15)

        tk.Label(hw_card, text="HARDWARE MATRIX", font=("Arial", 12, "bold"), fg="#94a3b8", bg="#334155").pack(anchor=tk.W)
        
        items = [
            ("Motors:", "2x 24V AT120 DC Gear (KM949299)"),
            ("Drivers:", "2x SmartElex 30S (30A PWM/DIR)"),
            ("Wheels:", "2x 100mm Heavy Duty HD Disc"),
            ("MCU:", "ESP32-S3-DevKitC-1 Wireless"),
            ("Display:", "10.1\" IPS 1280x800 Capacitive"),
            ("DDS Domain:", "ROS_DOMAIN_ID=42 (Isolated)")
        ]
        for k, v in items:
            f = tk.Frame(hw_card, bg="#334155")
            f.pack(fill=tk.X, pady=3)
            tk.Label(f, text=k, font=("Arial", 11, "bold"), fg="#38bdf8", bg="#334155", width=12, anchor=tk.W).pack(side=tk.LEFT)
            tk.Label(f, text=v, font=("Arial", 11), fg="#f8fafc", bg="#334155", anchor=tk.W).pack(side=tk.LEFT)

    def set_mode(self, mode_str):
        self.current_mode = mode_str
        if mode_str == "MANUAL TELEOP":
            self.btn_manual.configure(bg="#0284c7")
            self.btn_nav.configure(bg="#475569")
        else:
            self.btn_manual.configure(bg="#475569")
            self.btn_nav.configure(bg="#0284c7")

    def inc_speed(self):
        self.target_speed = min(1.05, self.target_speed + 0.05)
        self.speed_lbl.configure(text=f"Max: {self.target_speed:.2f} m/s")

    def dec_speed(self):
        self.target_speed = max(0.10, self.target_speed - 0.05)
        self.speed_lbl.configure(text=f"Max: {self.target_speed:.2f} m/s")

    def touch_drive(self, v, w):
        if self.estop_active:
            return
        self.ros_node.send_twist(v, w)

    def toggle_estop(self):
        self.estop_active = not self.estop_active
        if self.estop_active:
            self.btn_estop.configure(text="RESET\nESTOP", bg="#eab308", fg="#000000")
            self.ros_node.send_twist(0.0, 0.0)
        else:
            self.btn_estop.configure(text="EMERGENCY\nSTOP", bg="#dc2626", fg="white")

    def update_odom(self, x, y, v, w):
        self.odom_x = x
        self.odom_y = y
        self.actual_v = v
        self.actual_w = w

    def update_battery(self, volts, pct):
        self.batt_voltage = volts
        self.batt_pct = pct

    def periodic_ui_update(self):
        # Update clock
        self.time_lbl.configure(text=time.strftime("%H:%M:%S"))

        # Update battery UI
        self.batt_v_lbl.configure(text=f"{self.batt_voltage:.2f} V")
        pct = max(0.0, min(1.0, self.batt_pct))
        
        # Color code
        if pct > 0.60:
            color = "#22c55e"
        elif pct > 0.25:
            color = "#eab308"
        else:
            color = "#ef4444"

        w = self.batt_canvas.winfo_width()
        self.batt_canvas.coords(self.batt_bar, 0, 0, int(w * pct), 26)
        self.batt_canvas.itemconfig(self.batt_bar, fill=color)
        self.batt_pct_lbl.configure(text=f"{int(pct * 100)}% State of Charge (LiFePO4)")

        # Update velocity & pose
        self.vel_lbl.configure(text=f"v: {self.actual_v:+.2f} m/s\nω: {self.actual_w:+.2f} rad/s")
        self.pose_lbl.configure(text=f"X: {self.odom_x:+.2f} m\nY: {self.odom_y:+.2f} m")

        self.root.after(33, self.periodic_ui_update)

def main():
    rclpy.init()
    root = tk.Tk()
    app = TouchHMIApplication(root)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
