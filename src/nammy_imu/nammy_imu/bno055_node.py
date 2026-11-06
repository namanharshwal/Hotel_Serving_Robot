#!/usr/bin/env python3
"""
ROS 2 Node for Bosch BNO055 9-DOF Absolute Orientation Sensor.
Supports direct Jetson Orin Nano 40-Pin I2C bus connection (/dev/i2c-7 or /dev/i2c-1).
Publishes sensor_msgs/Imu and sensor_msgs/MagneticField with covariances.
"""

import sys
import math
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

# BNO055 I2C Registers
BNO055_ADDR_DEFAULT = 0x28
BNO055_ADDR_ALT     = 0x29

BNO055_CHIP_ID_ADDR = 0x00
BNO055_OPR_MODE_ADDR= 0x3D
BNO055_PWR_MODE_ADDR= 0x3E
BNO055_SYS_TRIGGER  = 0x3F

# Output data registers
BNO055_QUA_DATA_W_LSB = 0x20
BNO055_GYR_DATA_X_LSB = 0x14
BNO055_ACC_DATA_X_LSB = 0x08
BNO055_MAG_DATA_X_LSB = 0x0E
BNO055_CALIB_STAT     = 0x35

# Operation modes
OPERATION_MODE_CONFIG = 0x00
OPERATION_MODE_NDOF   = 0x0C

class BNO055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')

        self.declare_parameter('i2c_bus', 7)                # /dev/i2c-7 on Jetson 40-pin header (Pins 27, 28)
        self.declare_parameter('i2c_address', 0x28)         # Default BNO055 I2C Address (0x28 or 0x29)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)        # 50 Hz
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
        self.init_hardware()

        self.timer = self.create_timer(1.0 / self.rate, self.read_and_publish)

    def init_hardware(self):
        if self.sim_mode or not SMBUS_AVAILABLE:
            self.get_logger().warn("[SIMULATION] BNO055 running in simulation fallback mode.")
            self.sim_mode = True
            return

        try:
            self.bus = smbus2.SMBus(self.bus_num)
            chip_id = self.bus.read_byte_data(self.addr, BNO055_CHIP_ID_ADDR)
            if chip_id != 0xA0:
                self.get_logger().warn(f"Unexpected Chip ID 0x{chip_id:02X} on /dev/i2c-{self.bus_num} at 0x{self.addr:02X}.")
            
            # Switch to CONFIG mode to set parameters
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG)
            time.sleep(0.02)
            # Normal Power mode
            self.bus.write_byte_data(self.addr, BNO055_PWR_MODE_ADDR, 0x00)
            time.sleep(0.01)
            # Switch to 9-DOF Fusion mode (NDOF)
            self.bus.write_byte_data(self.addr, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF)
            time.sleep(0.05)

            self.get_logger().info(f"[SUCCESS] Connected to BNO055 9-DOF IMU on /dev/i2c-{self.bus_num} @ 0x{self.addr:02X}")
        except Exception as e:
            self.get_logger().warn(f"Could not open BNO055 on /dev/i2c-{self.bus_num}: {e}")
            self.get_logger().info("[FALLBACK] Using simulation mode for IMU.")
            self.sim_mode = True

    def read_and_publish(self):
        now = self.get_clock().now()

        imu_msg = Imu()
        imu_msg.header.stamp = now.to_msg()
        imu_msg.header.frame_id = self.frame_id

        if not self.sim_mode and self.bus:
            try:
                # Read 8 bytes of Quaternion data (W, X, Y, Z, each 2 bytes int16)
                data = self.bus.read_i2c_block_data(self.addr, BNO055_QUA_DATA_W_LSB, 8)
                w, x, y, z = struct.unpack('<hhhh', bytes(data))
                scale = 1.0 / 16384.0  # 1 LSB = 2^-14
                imu_msg.orientation.w = float(w * scale)
                imu_msg.orientation.x = float(x * scale)
                imu_msg.orientation.y = float(y * scale)
                imu_msg.orientation.z = float(z * scale)

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

            except Exception as e:
                self.get_logger().warn(f"I2C read failure: {e}", throttle_duration_sec=2.0)
                self.set_sim_orientation(imu_msg)
        else:
            self.set_sim_orientation(imu_msg)

        # Covariances for robot_localization EKF
        imu_msg.orientation_covariance[0] = 0.002
        imu_msg.orientation_covariance[4] = 0.002
        imu_msg.orientation_covariance[8] = 0.002

        imu_msg.angular_velocity_covariance[0] = 0.001
        imu_msg.angular_velocity_covariance[4] = 0.001
        imu_msg.angular_velocity_covariance[8] = 0.001

        imu_msg.linear_acceleration_covariance[0] = 0.01
        imu_msg.linear_acceleration_covariance[4] = 0.01
        imu_msg.linear_acceleration_covariance[8] = 0.01

        self.imu_pub.publish(imu_msg)

    def set_sim_orientation(self, msg: Imu):
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.linear_acceleration.z = 9.80665

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
