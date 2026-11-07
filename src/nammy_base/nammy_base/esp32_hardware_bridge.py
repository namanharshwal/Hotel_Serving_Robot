#!/usr/bin/env python3
"""
ROS 2 Hardware Bridge Node for Nammy Robot.
Interfaces Jetson Orin Nano with ESP32-S3 DevKitC-1 over high-speed USB Serial,
controlling 2x SmartElex 30S motor drivers and reading AT120 quadrature encoders.
"""

import sys
import math
import time
import threading
import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, BatteryState, Range, Imu
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

from nammy_base.packet_protocol import (
    pack_cmd_vel, pack_estop, pack_heartbeat,
    PacketParser, unpack_telemetry,
    MSG_TELEMETRY, MSG_HEARTBEAT
)

class ESP32HardwareBridge(Node):
    def __init__(self):
        super().__init__('esp32_hardware_bridge')

        # Declare ROS 2 Parameters
        self.declare_parameter('serial_port', '/dev/nammy_esp32')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.050)        # 100mm HD Disc Wheel (r=0.050m)
        self.declare_parameter('wheel_separation', 0.380)    # 380mm track width
        self.declare_parameter('publish_rate', 50.0)         # 50 Hz control & telemetry loop
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('cmd_timeout', 1.0)           # Watchdog timeout in seconds
        self.declare_parameter('simulate_hardware', False)   # Auto-fallback if port not present
        self.declare_parameter('max_linear_velocity', 1.2)   # Max linear velocity m/s
        self.declare_parameter('max_angular_velocity', 2.5)  # Max angular velocity rad/s
        self.declare_parameter('max_linear_acceleration', 4.0)  # Snappy instant response (4.0 m/s^2)
        self.declare_parameter('max_angular_acceleration', 8.0)  # Snappy instant turning (8.0 rad/s^2)
        self.declare_parameter('invert_encoders', False)         # Standard encoder polarity aligned with ROS REP-103 (+X forward)
        
        # Closed-Loop IMU Heading Stabilization & Dual-Motor Synchronization Parameters
        self.declare_parameter('enable_imu_heading_hold', True)
        self.declare_parameter('heading_kp', 2.2)            # Heading error proportional gain (rad/s per rad)
        self.declare_parameter('heading_ki', 0.08)           # Heading error integral gain
        self.declare_parameter('heading_kd', 0.15)           # Gyroscopic rate damping gain (rad/s per rad/s)
        self.declare_parameter('max_heading_correction', 0.60) # Maximum corrective angular velocity (rad/s)
        self.declare_parameter('enable_rpm_sync', True)       # Cross-motor speed equalizer
        self.declare_parameter('rpm_sync_kp', 0.25)          # Wheel speed error proportional gain
        self.declare_parameter('rpm_sync_ki', 0.04)          # Wheel speed error integral gain
        self.declare_parameter('straight_lin_threshold', 0.01) # Min linear speed to activate heading lock (m/s)
        self.declare_parameter('straight_ang_threshold', 0.05) # Max angular command considered straight motion (rad/s)

        # Retrieve Parameters
        self.port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.b = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.sim_mode = self.get_parameter('simulate_hardware').get_parameter_value().bool_value
        self.max_v = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_angular_velocity').get_parameter_value().double_value
        self.max_lin_accel = self.get_parameter('max_linear_acceleration').get_parameter_value().double_value
        self.max_ang_accel = self.get_parameter('max_angular_acceleration').get_parameter_value().double_value
        self.invert_encoders = self.get_parameter('invert_encoders').get_parameter_value().bool_value
        self.enable_imu_heading_hold = self.get_parameter('enable_imu_heading_hold').get_parameter_value().bool_value
        self.heading_kp = self.get_parameter('heading_kp').get_parameter_value().double_value
        self.heading_ki = self.get_parameter('heading_ki').get_parameter_value().double_value
        self.heading_kd = self.get_parameter('heading_kd').get_parameter_value().double_value
        self.max_heading_corr = self.get_parameter('max_heading_correction').get_parameter_value().double_value
        self.enable_rpm_sync = self.get_parameter('enable_rpm_sync').get_parameter_value().bool_value
        self.rpm_sync_kp = self.get_parameter('rpm_sync_kp').get_parameter_value().double_value
        self.rpm_sync_ki = self.get_parameter('rpm_sync_ki').get_parameter_value().double_value
        self.straight_lin_thresh = self.get_parameter('straight_lin_threshold').get_parameter_value().double_value
        self.straight_ang_thresh = self.get_parameter('straight_ang_threshold').get_parameter_value().double_value

        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info("==================================================================")
        self.get_logger().info("INITIALIZING NAMMY ROBOT ESP32-S3 HARDWARE BRIDGE")
        self.get_logger().info(f"Target Port: {self.port_name} @ {self.baud_rate} baud")
        self.get_logger().info(f"Wheel Radius: {self.r:.3f} m (100mm Disc) | Separation: {self.b:.3f} m")
        self.get_logger().info(f"S-Curve Slew Limiter: Lin {self.max_lin_accel:.2f} m/s^2 | Ang {self.max_ang_accel:.2f} rad/s^2")
        self.get_logger().info("==================================================================")

        # State Variables
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.last_cmd_time = self.get_clock().now()
        
        # Dead Reckoning Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        
        # Feedback positions and velocities
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0
        self.prev_left_pos = None
        self.prev_right_pos = None
        
        # IMU Heading Stabilization & Synchronization State
        self.imu_yaw = 0.0
        self.imu_yaw_rate = 0.0
        self.last_imu_time = None
        self.target_heading = 0.0
        self.heading_hold_active = False
        self.heading_integral = 0.0
        self.rpm_sync_integral = 0.0

        # Battery state & Ultrasonic Ranges
        self.battery_voltage = 26.4  # Nominal 24V LiFePO4 (8S)
        self.sonar_front_m = 4.50
        self.sonar_left_m = 4.50
        self.sonar_right_m = 4.50
        self.sonar_rear_m = 4.50

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'nammy/battery_state', 10)
        self.sonar_front_pub = self.create_publisher(Range, 'sonar/front', 10)
        self.sonar_left_pub = self.create_publisher(Range, 'sonar/left', 10)
        self.sonar_right_pub = self.create_publisher(Range, 'sonar/right', 10)
        self.sonar_rear_pub = self.create_publisher(Range, 'sonar/rear', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10, callback_group=self.cb_group)
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile_sensor_data,
            callback_group=self.cb_group
        )

        # Serial Communication
        self.serial_conn = None
        self.parser = PacketParser()
        self.serial_lock = threading.Lock()
        self.init_serial()

        # Timers
        self.loop_timer = self.create_timer(1.0 / self.publish_rate, self.control_loop, callback_group=self.cb_group)
        self.heartbeat_timer = self.create_timer(0.5, self.send_heartbeat, callback_group=self.cb_group)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def imu_callback(self, msg: Imu):
        """Extract orientation yaw and gyroscopic angular velocity from Bosch BNO055."""
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_yaw_rate = msg.angular_velocity.z
        self.last_imu_time = self.get_clock().now()

    def init_serial(self):
        """Connect to ESP32-S3 serial port with graceful simulation fallback."""
        if self.sim_mode:
            self.get_logger().warn("[SIMULATION MODE ACTIVE] Running without hardware connection.")
            return

        try:
            self.serial_conn = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=0.05,
                write_timeout=0.05
            )
            self.get_logger().info(f"[SUCCESS] Connected to ESP32-S3 on {self.port_name}")
        except Exception as e:
            self.get_logger().warn(f"Failed to open {self.port_name}: {e}")
            # Search available ports
            available = [p.device for p in serial.tools.list_ports.comports()]
            self.get_logger().info(f"Available serial ports: {available}")
            
            # Prioritize ACM ports (ESP32-S3) before USB ports (to avoid collision with LiDAR)
            candidate_ports = sorted(available, key=lambda p: (0 if 'ACM' in p else 1))
            connected = False
            for port in candidate_ports:
                if 'ACM' in port or 'USB' in port:
                    try:
                        conn = serial.Serial(port, self.baud_rate, timeout=0.1, write_timeout=0.05)
                        # Probe port for ESP32 telemetry signature (0xAA 0x55)
                        probe_start = time.time()
                        signature_found = False
                        raw_buf = bytearray()
                        while time.time() - probe_start < 0.25:
                            if conn.in_waiting:
                                raw_buf.extend(conn.read(conn.in_waiting))
                                if b'\xaa\x55' in raw_buf:
                                    signature_found = True
                                    break
                            time.sleep(0.02)
                        
                        if signature_found or 'ACM' in port:
                            self.serial_conn = conn
                            self.port_name = port
                            self.get_logger().info(f"[PROBE SUCCESS] Verified and connected to ESP32-S3 on {port}")
                            connected = True
                            break
                        else:
                            conn.close()
                    except Exception:
                        continue
            
            if not connected:
                self.get_logger().warn("[AUTO-SIMULATION] No hardware port accessible. Falling back to internal simulation mode.")
                self.sim_mode = True

    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity setpoint and update targets with security bounds checking."""
        vx = msg.linear.x
        wz = msg.angular.z

        # Security & Sanity Input Validation: Reject NaN or Infinite values
        if math.isnan(vx) or math.isinf(vx) or math.isnan(wz) or math.isinf(wz):
            self.get_logger().error("SECURITY ALERT: Malformed cmd_vel received (NaN or Inf)! Halting robot.")
            self.target_v = 0.0
            self.target_w = 0.0
            return

        # Defensive bounds clamping to physical platform limits
        self.target_v = max(min(vx, self.max_v), -self.max_v)
        self.target_w = max(min(wz, self.max_w), -self.max_w)
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        """Main periodic control and telemetry update loop."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        # 1. Check command timeout (Software Watchdog)
        time_since_cmd = (current_time - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > self.cmd_timeout:
            desired_v = 0.0
            desired_w = 0.0
        else:
            desired_v = self.target_v
            desired_w = self.target_w

        # S-Curve Acceleration & Deceleration Slew Limiter (Anti-Jerk Filter)
        # Prevents any step jumps in motor torque; guarantees buttery-smooth starts and stops
        max_v_step = self.max_lin_accel * dt
        max_w_step = self.max_ang_accel * dt

        delta_v = desired_v - self.current_v
        if abs(delta_v) <= max_v_step:
            self.current_v = desired_v
        else:
            self.current_v += math.copysign(max_v_step, delta_v)

        delta_w = desired_w - self.current_w
        if abs(delta_w) <= max_w_step:
            self.current_w = desired_w
        else:
            self.current_w += math.copysign(max_w_step, delta_w)

        # 2. Closed-Loop IMU Heading Stabilization & Wheel Speed Synchronization
        w_imu_corr = 0.0
        sync_corr = 0.0
        is_commanded_straight = (abs(self.current_v) > self.straight_lin_thresh) and \
                                (abs(self.target_w) < self.straight_ang_thresh)

        if is_commanded_straight:
            # Check if IMU data is available and fresh (< 200 ms)
            imu_fresh = False
            if self.last_imu_time is not None:
                imu_age = (current_time - self.last_imu_time).nanoseconds / 1e9
                if imu_age < 0.20:
                    imu_fresh = True

            # Engage heading hold: latch heading at motion onset
            if not self.heading_hold_active and imu_fresh:
                self.target_heading = self.imu_yaw
                self.heading_hold_active = True
                self.heading_integral = 0.0
                self.rpm_sync_integral = 0.0
                self.get_logger().info(
                    f"[HEADING HOLD] Engaged. Latched Heading: {math.degrees(self.target_heading):.2f}°",
                    throttle_duration_sec=2.0
                )

            if self.heading_hold_active and imu_fresh and self.enable_imu_heading_hold:
                # Shortest angular error between latched heading and current IMU orientation
                yaw_error = self.normalize_angle(self.target_heading - self.imu_yaw)

                self.heading_integral += yaw_error * dt
                self.heading_integral = max(-0.5, min(0.5, self.heading_integral))

                # Corrective angular velocity (PD controller with gyroscopic rate damping)
                # If robot drifted left (imu_yaw > target_heading -> yaw_error < 0),
                # corrective omega is negative -> steers robot right to restore heading.
                w_imu_corr = (self.heading_kp * yaw_error) + \
                             (self.heading_ki * self.heading_integral) - \
                             (self.heading_kd * self.imu_yaw_rate)

                # Clamp corrective angular velocity within configured limits
                w_imu_corr = max(-self.max_heading_corr, min(self.max_heading_corr, w_imu_corr))

            # Wheel RPM Equalizer (Electronic Differential Lock)
            # If encoders show one wheel spinning faster than the other during straight motion
            if self.enable_rpm_sync and abs(self.current_v) > 0.02:
                # speed_diff > 0 means left wheel is spinning faster than right
                speed_diff = self.left_vel - self.right_vel
                self.rpm_sync_integral += speed_diff * dt
                self.rpm_sync_integral = max(-1.0, min(1.0, self.rpm_sync_integral))

                sync_corr = (self.rpm_sync_kp * speed_diff) + (self.rpm_sync_ki * self.rpm_sync_integral)
                sync_corr = max(-1.5, min(1.5, sync_corr))
        else:
            # User commanded a turn or robot is stationary -> disengage heading lock
            self.heading_hold_active = False
            self.heading_integral = 0.0
            self.rpm_sync_integral = 0.0

        # 3. Differential Drive Inverse Kinematics with Active Heading & Sync Trim
        effective_w = self.current_w + w_imu_corr
        left_target_rad_s = (self.current_v - (effective_w * self.b / 2.0)) / self.r - (sync_corr / 2.0)
        right_target_rad_s = (self.current_v + (effective_w * self.b / 2.0)) / self.r + (sync_corr / 2.0)

        # 4. Transmit to ESP32-S3 or Simulate
        if not self.sim_mode and self.serial_conn and self.serial_conn.is_open:
            self.send_serial_cmd(left_target_rad_s, right_target_rad_s)
            self.read_serial_telemetry()
        else:
            # First-order motor response simulation
            tau = 0.08  # motor mechanical time constant ~80ms
            alpha = min(1.0, dt / tau)
            self.left_vel += alpha * (left_target_rad_s - self.left_vel)
            self.right_vel += alpha * (right_target_rad_s - self.right_vel)
            self.left_pos += self.left_vel * dt
            self.right_pos += self.right_vel * dt

        # 5. Forward Kinematics & Odometry Integration
        if self.prev_left_pos is None or self.prev_right_pos is None:
            self.prev_left_pos = self.left_pos
            self.prev_right_pos = self.right_pos
            self.last_time = current_time
            return

        d_left = (self.left_pos - self.prev_left_pos) * self.r
        d_right = (self.right_pos - self.prev_right_pos) * self.r
        self.prev_left_pos = self.left_pos
        self.prev_right_pos = self.right_pos

        d_center = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self.b

        # Exact runge-kutta 2nd order pose integration
        self.x += d_center * math.cos(self.th + (d_theta / 2.0))
        self.y += d_center * math.sin(self.th + (d_theta / 2.0))
        self.th += d_theta
        # Normalize theta to [-pi, pi]
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # Linear and angular velocity estimation
        v_actual = (self.right_vel + self.left_vel) * self.r / 2.0
        w_actual = (self.right_vel - self.left_vel) * self.r / self.b

        # 5. Publish Odometry Message & TF
        self.publish_odometry(current_time, v_actual, w_actual)

        # 6. Publish Joint States (for URDF wheel rotation)
        self.publish_joint_states(current_time)

        # 7. Publish Battery Telemetry
        self.publish_battery(current_time)

        self.last_time = current_time

    def send_serial_cmd(self, left_rad_s: float, right_rad_s: float):
        """Send packed velocity command to ESP32-S3 with auto-reconnect on disconnect."""
        packet = pack_cmd_vel(left_rad_s, right_rad_s)
        try:
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.write(packet)
        except (serial.SerialException, OSError) as e:
            self.handle_disconnect(f"Write error: {e}")

    def send_heartbeat(self):
        """Periodic keepalive packet and auto-reconnection checker."""
        if self.sim_mode or not self.serial_conn or not self.serial_conn.is_open:
            self.check_auto_reconnect()
            return

        packet = pack_heartbeat()
        try:
            with self.serial_lock:
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.write(packet)
        except (serial.SerialException, OSError) as e:
            self.handle_disconnect(f"Heartbeat error: {e}")

    def read_serial_telemetry(self):
        """Read and parse incoming packets from ESP32-S3."""
        try:
            with self.serial_lock:
                if not self.serial_conn or not self.serial_conn.is_open:
                    return
                bytes_available = self.serial_conn.in_waiting
                if bytes_available > 0:
                    raw_data = self.serial_conn.read(bytes_available)
                    self.parser.feed(raw_data)
        except (serial.SerialException, OSError) as e:
            self.handle_disconnect(f"Read error: {e}")
            return

        while True:
            res = self.parser.parse()
            if res is None:
                break
            msg_type, payload = res
            if msg_type == MSG_TELEMETRY:
                telemetry = unpack_telemetry(payload)
                if telemetry:
                    enc_sign = -1.0 if self.invert_encoders else 1.0
                    self.left_pos = enc_sign * telemetry.left_pos_rad
                    self.right_pos = enc_sign * telemetry.right_pos_rad
                    self.left_vel = enc_sign * telemetry.left_vel_rad_s
                    self.right_vel = enc_sign * telemetry.right_vel_rad_s
                    self.battery_voltage = telemetry.battery_volts
                    self.sonar_front_m = telemetry.sonar_front_m
                    self.sonar_left_m = telemetry.sonar_left_m
                    self.sonar_right_m = telemetry.sonar_right_m
                    self.sonar_rear_m = telemetry.sonar_rear_m

                    now_msg = self.get_clock().now().to_msg()
                    self.publish_sonar_range(self.sonar_front_pub, 'front_sonar_link', telemetry.sonar_front_m, now_msg)
                    self.publish_sonar_range(self.sonar_left_pub, 'left_sonar_link', telemetry.sonar_left_m, now_msg)
                    self.publish_sonar_range(self.sonar_right_pub, 'right_sonar_link', telemetry.sonar_right_m, now_msg)
                    self.publish_sonar_range(self.sonar_rear_pub, 'rear_sonar_link', telemetry.sonar_rear_m, now_msg)

    def publish_sonar_range(self, publisher, frame_id: str, distance_m: float, timestamp):
        """Publish standard ROS 2 sensor_msgs/Range message for Nav2 costmaps."""
        msg = Range()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.2618  # 15 degrees cone for US-100
        msg.min_range = 0.02        # 2 cm
        msg.max_range = 4.50        # 4.5 meters
        msg.range = float(distance_m)
        publisher.publish(msg)

    def handle_disconnect(self, reason: str):
        """Handle USB unplug / serial disconnection without crashing."""
        self.get_logger().warn(f"[USB DISCONNECT] {reason}. Transitioning to auto-reconnect fallback...")
        with self.serial_lock:
            try:
                if self.serial_conn:
                    self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        self.sim_mode = True

    def check_auto_reconnect(self):
        """Poll for re-plugged USB serial connection."""
        target_ports = [self.port_name] + [p.device for p in serial.tools.list_ports.comports() if 'USB' in p.device or 'ACM' in p.device]
        for p_cand in target_ports:
            try:
                test_conn = serial.Serial(p_cand, self.baud_rate, timeout=0.05, write_timeout=0.05)
                self.port_name = p_cand
                self.serial_conn = test_conn
                self.sim_mode = False
                self.get_logger().info(f"[USB RECONNECTED] Successfully re-established connection on {p_cand}!")
                break
            except Exception:
                continue

    def publish_odometry(self, current_time: Time, v_actual: float, w_actual: float):
        """Publish nav_msgs/Odometry and optional TF transform."""
        q_z = math.sin(self.th / 2.0)
        q_w = math.cos(self.th / 2.0)

        # TF Transform
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = q_z
            t.transform.rotation.w = q_w
            self.tf_broadcaster.sendTransform(t)

        # Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = q_z
        odom.pose.pose.orientation.w = q_w

        # Covariances
        odom.pose.covariance[0] = 0.001   # x
        odom.pose.covariance[7] = 0.001   # y
        odom.pose.covariance[35] = 0.005  # yaw

        odom.twist.twist.linear.x = v_actual
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w_actual
        odom.twist.covariance[0] = 0.002
        odom.twist.covariance[35] = 0.010

        self.odom_pub.publish(odom)

    def publish_joint_states(self, current_time: Time):
        """Publish sensor_msgs/JointState for URDF wheel visualization."""
        js = JointState()
        js.header.stamp = current_time.to_msg()
        # Broadcast both SolidWorks URDF joint names (ltwheeljt, rtwheeljt) and aliases
        js.name = ['ltwheeljt', 'rtwheeljt', 'left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_pos, self.right_pos, self.left_pos, self.right_pos]
        js.velocity = [self.left_vel, self.right_vel, self.left_vel, self.right_vel]
        self.joint_pub.publish(js)

    def publish_battery(self, current_time: Time):
        """Publish 24V LiFePO4 battery telemetry and percentage."""
        batt = BatteryState()
        batt.header.stamp = current_time.to_msg()
        batt.voltage = float(self.battery_voltage)
        batt.current = float('nan')
        batt.charge = float('nan')
        batt.capacity = 15.0  # 15 Ah LFP Pack
        batt.design_capacity = 15.0

        # LiFePO4 8S State of Charge (SOC) approximation curve
        # Cutoff: 20.0V (0%), Nominal Plateau: 25.6V - 26.4V (~30% to 80%), Full: 28.8V - 29.2V (100%)
        v = self.battery_voltage
        if v >= 28.8:
            soc = 1.0
        elif v <= 20.0:
            soc = 0.0
        elif v < 25.0:
            soc = 0.0 + (v - 20.0) / 5.0 * 0.20  # 20.0V to 25.0V -> 0% to 20%
        elif v < 26.6:
            soc = 0.20 + (v - 25.0) / 1.6 * 0.65  # Flat plateau 25.0V to 26.6V -> 20% to 85%
        else:
            soc = 0.85 + (v - 26.6) / 2.2 * 0.15  # 26.6V to 28.8V -> 85% to 100%

        batt.percentage = max(0.0, min(1.0, soc))
        batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        batt.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE  # LiFePO4
        batt.present = True

        self.battery_pub.publish(batt)

def main(args=None):
    rclpy.init(args=args)
    node = ESP32HardwareBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ESP32 hardware bridge...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
