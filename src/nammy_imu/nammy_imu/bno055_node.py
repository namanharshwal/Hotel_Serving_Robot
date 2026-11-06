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

        self.declare_parameter('i2c_bus', 7)                # /dev/i2c-7 on Jetson 40-pin header (Pins 27, 28)
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

        if not self.sim_mode:
            self.init_hardware()

        self.timer = self.create_timer(1.0 / self.rate, self.read_and_publish)

    def init_hardware(self):
        if not SMBUS_AVAILABLE:
            self.get_logger().warn("[BNO055] smbus2 library not available.")
            self.hardware_ready = False
            return False

        try:
            self.bus = smbus2.SMBus(self.bus_num)
            chip_id = self.bus.read_byte_data(self.addr, BNO055_CHIP_ID_ADDR)
            if chip_id != 0xA0:
                self.get_logger().warn(f"Unexpected Chip ID 0x{chip_id:02X} on /dev/i2c-{self.bus_num} at 0x{self.addr:02X}.")

            # 1. Switch to CONFIG mode to configure chip
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
            time.sleep(0.025)

            # 2. Set Normal Power mode
            self.bus.write_byte_data(self.addr, BNO055_PWR_MODE_ADDR, 0x00)
            time.sleep(0.010)

            # 3. Switch to IMU mode (0x08) - relative Gyro + Accel fusion matching hover_ws
            # This completely avoids magnetic distortion from AT120 high-current motors
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMU)
            time.sleep(0.030)

            self.get_logger().info(f"[SUCCESS] Connected to Bosch BNO055 IMU on /dev/i2c-{self.bus_num} @ 0x{self.addr:02X} (Mode: IMU 0x08)")
            self.hardware_ready = True
            return True
        except Exception as e:
            self.get_logger().warn(f"Could not connect to BNO055 on /dev/i2c-{self.bus_num} @ 0x{self.addr:02X}: {e}")
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
                imu_msg.orientation.w = float(w / norm)
                imu_msg.orientation.x = float(x / norm)
                imu_msg.orientation.y = float(y / norm)
                imu_msg.orientation.z = float(z / norm)
            else:
                return

            # Read Angular Velocity (Gyro: X, Y, Z int16, 1 LSB = 1/16 rad/s)
            gyr_data = self.bus.read_i2c_block_data(self.addr, BNO055_GYR_DATA_X_LSB, 6)
            gx, gy, gz = struct.unpack('<hhh', bytes(gyr_data))
            imu_msg.angular_velocity.x = float(gx / 900.0)
            imu_msg.angular_velocity.y = float(gy / 900.0)
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
            self.get_logger().warn(f"BNO055 read glitch: {e}", throttle_duration_sec=3.0)
            self.hardware_ready = False

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
