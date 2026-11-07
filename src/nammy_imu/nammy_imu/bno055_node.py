#!/usr/bin/env python3
"""
Bosch BNO055 9-DOF Absolute Orientation IMU Driver Node for Nammy Robot.
Configured for real-time IMU sensor fusion matching proven hover_ws architecture.
Operates in IMU mode (0x08) for gyro + accelerometer relative fusion without magnetic distortion.

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
"""

import time
import struct
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

# Bosch BNO055 Register Map
BNO055_CHIP_ID_ADDR = 0x00
BNO055_PAGE_ID_ADDR = 0x07
BNO055_OPR_MODE_ADDR = 0x3D
BNO055_PWR_MODE_ADDR = 0x3E
BNO055_SYS_TRIGGER_ADDR = 0x3F

# Output data registers
BNO055_ACC_DATA_X_LSB = 0x08
BNO055_MAG_DATA_X_LSB = 0x0E
BNO055_GYR_DATA_X_LSB = 0x14
BNO055_QUA_DATA_W_LSB = 0x20
BNO055_CALIB_STAT     = 0x35

# Operation modes (Matching hover_ws production setup)
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_IMU    = 0x08  # Relative orientation fusion (Gyro + Accel, NO magnetic distortion)
OPERATION_MODE_NDOF   = 0x0C

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')

        self.declare_parameter('i2c_bus', 1)                # /dev/i2c-1 on Jetson 40-pin header (Pins 3, 5)
        self.declare_parameter('i2c_address', 0x28)         # Default BNO055 I2C Address (0x28 = 40)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)       # 100 Hz matching hover_ws
        self.declare_parameter('simulate_hardware', False)

        self.bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('publish_rate').value
        self.sim_mode = self.get_parameter('simulate_hardware').value

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.calib_pub = self.create_publisher(String, 'imu/calibration', 10)

        self.bus = None
        self.hardware_ready = False
        self.last_reconnect_attempt = 0.0
        self.initial_yaw = None

        if not self.sim_mode:
            self.init_hardware()

        self.timer = self.create_timer(1.0 / self.rate, self.read_and_publish)

    def init_hardware(self):
        if not SMBUS_AVAILABLE:
            self.get_logger().warn("[BNO055] smbus2 library not available.")
            self.hardware_ready = False
            return False

        # Ensure previous bus file descriptor is released
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None

        # Candidate user header buses and addresses (7: Pin 27/28, 1: Pin 3/5)
        # Note: Probing internal Jetson PMIC buses (0, 2, 4, 5) is strictly forbidden
        candidate_buses = [self.bus_num]
        for b in [7, 1]:
            if b not in candidate_buses:
                candidate_buses.append(b)

        candidate_addrs = [self.addr]
        for a in [0x28, 0x29]:
            if a not in candidate_addrs:
                candidate_addrs.append(a)

        for b_cand in candidate_buses:
            for a_cand in candidate_addrs:
                new_bus = None
                try:
                    new_bus = smbus2.SMBus(b_cand)
                    chip_id = new_bus.read_byte_data(a_cand, BNO055_CHIP_ID_ADDR)
                    if chip_id != 0xA0:
                        new_bus.close()
                        continue

                    # 1. Switch to CONFIG mode to configure chip
                    new_bus.write_byte_data(a_cand, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
                    time.sleep(0.025)

                    # 2. Set Normal Power mode
                    new_bus.write_byte_data(a_cand, BNO055_PWR_MODE_ADDR, 0x00)
                    time.sleep(0.010)

                    # 3. Switch to IMU mode (0x08) - relative Gyro + Accel fusion matching hover_ws
                    new_bus.write_byte_data(a_cand, BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMU)
                    time.sleep(0.030)

                    self.bus = new_bus
                    self.bus_num = b_cand
                    self.addr = a_cand
                    self.get_logger().info(f"[SUCCESS] Connected to Bosch BNO055 IMU on /dev/i2c-{self.bus_num} @ 0x{self.addr:02X} (Mode: IMU 0x08)")
                    self.hardware_ready = True
                    return True
                except Exception:
                    if new_bus is not None:
                        try:
                            new_bus.close()
                        except Exception:
                            pass
                    continue

        self.bus = None
        self.get_logger().warn(f"Could not find BNO055 on candidate buses {candidate_buses} at addresses {[hex(a) for a in candidate_addrs]}.", throttle_duration_sec=10.0)
        self.hardware_ready = False
        return False

    def read_and_publish(self):
        now = self.get_clock().now()

        # If hardware is not connected, attempt periodic reconnection but DO NOT publish fake data
        if not self.hardware_ready:
            current_time = time.time()
            if current_time - self.last_reconnect_attempt > 2.0:
                self.last_reconnect_attempt = current_time
                if self.init_hardware():
                    self.get_logger().info("[BNO055] Successfully recovered connection!")
            # IMPORTANT: Do not publish fake 0.0 orientation to /imu/data!
            # When /imu/data is silent, robot_localization EKF seamlessly fuses wheel odometry alone,
            # allowing full, natural body movement and turns without being artificially locked to zero.
            return

        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = self.frame_id

        try:
            # Read 8 bytes of fused Quaternion data (W, X, Y, Z, each int16)
            data = self.bus.read_i2c_block_data(self.addr, BNO055_QUA_DATA_W_LSB, 8)
            w, x, y, z = struct.unpack('<hhhh', bytes(data))
            
            norm = (w*w + x*x + y*y + z*z) ** 0.5
            if norm > 0.0:
                qw = float(w / norm)
                qx = float(x / norm)
                qy = float(y / norm)
                qz = float(z / norm)

                # Compute Euler angles from fused quaternion
                siny_cosp = 2.0 * (qw * qz + qx * qy)
                cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
                raw_yaw = math.atan2(siny_cosp, cosy_cosp)

                sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
                pitch = math.asin(sinp)

                sinr_cosp = 2.0 * (qw * qx + qy * qz)
                cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
                roll = math.atan2(sinr_cosp, cosr_cosp)

                # Auto-zero initial heading tare offset so robot forward heading starts at 0.00 deg
                if self.initial_yaw is None:
                    self.initial_yaw = raw_yaw
                    self.get_logger().info(f"[BNO055] Auto-zeroed initial heading tare offset: {raw_yaw * 57.2958:.2f}° -> 0.00° (eliminates sideways motion)")

                aligned_yaw = raw_yaw - self.initial_yaw
                aligned_yaw = math.atan2(math.sin(aligned_yaw), math.cos(aligned_yaw))

                # Reconstruct aligned quaternion (zero yaw at startup)
                cy = math.cos(aligned_yaw * 0.5)
                sy = math.sin(aligned_yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)

                imu_msg.orientation.w = float(cr * cp * cy + sr * sp * sy)
                imu_msg.orientation.x = float(sr * cp * cy - cr * sp * sy)
                imu_msg.orientation.y = float(cr * sp * cy + sr * cp * sy)
                imu_msg.orientation.z = float(cr * cp * sy - sr * sp * cy)
            else:
                return

            # Read Angular Velocity (Gyro: X, Y, Z int16, 1 LSB = 1/16 rad/s)
            # Z-axis angular velocity (+gz) matches left/right yaw rate (CCW positive)
            gyr_data = self.bus.read_i2c_block_data(self.addr, BNO055_GYR_DATA_X_LSB, 6)
            gx, gy, gz = struct.unpack('<hhh', bytes(gyr_data))
            imu_msg.angular_velocity.x = float(-gx / 900.0)
            imu_msg.angular_velocity.y = float(-gy / 900.0)
            imu_msg.angular_velocity.z = float(gz / 900.0)

            # Read Linear Acceleration (Accel: X, Y, Z int16, 1 LSB = 1/100 m/s^2)
            acc_data = self.bus.read_i2c_block_data(self.addr, BNO055_ACC_DATA_X_LSB, 6)
            ax, ay, az = struct.unpack('<hhh', bytes(acc_data))
            imu_msg.linear_acceleration.x = float(ax / 100.0)
            imu_msg.linear_acceleration.y = float(ay / 100.0)
            imu_msg.linear_acceleration.z = float(az / 100.0)

            # Standard covariance matching hover_ws
            imu_msg.orientation_covariance[0] = 0.05
            imu_msg.orientation_covariance[4] = 0.05
            imu_msg.orientation_covariance[8] = 0.05

            imu_msg.angular_velocity_covariance[0] = 0.01
            imu_msg.angular_velocity_covariance[4] = 0.01
            imu_msg.angular_velocity_covariance[8] = 0.01

            imu_msg.linear_acceleration_covariance[0] = 0.1
            imu_msg.linear_acceleration_covariance[4] = 0.1
            imu_msg.linear_acceleration_covariance[8] = 0.1

            self.imu_pub.publish(imu_msg)

        except Exception as e:
            self.get_logger().warn(f"BNO055 read glitch: {e}", throttle_duration_sec=5.0)
            if self.bus is not None:
                try:
                    self.bus.close()
                except Exception:
                    pass
                self.bus = None
            self.hardware_ready = False

    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except Exception:
                pass
            self.bus = None
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
