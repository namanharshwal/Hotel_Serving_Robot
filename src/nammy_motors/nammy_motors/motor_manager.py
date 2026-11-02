#!/usr/bin/env python3
"""
Motor Manager Node for Nammy Robot.
Manages SmartElex 30S driver parameters, acceleration profiles, and AT120 motor health.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

class MotorManager(Node):
    def __init__(self):
        super().__init__('motor_manager')

        self.declare_parameter('max_wheel_rpm', 200.0)      # AT120 rated speed ~200 RPM
        self.declare_parameter('gear_ratio', 15.0)          # 15:1 gearbox
        self.declare_parameter('continuous_current_a', 30.0)# SmartElex 30S 30A rating
        self.declare_parameter('peak_current_a', 85.0)      # Peak 85A
        self.declare_parameter('deadband_compensation', 60) # PWM duty offset

        self.max_rpm = self.get_parameter('max_wheel_rpm').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.max_rad_s = self.max_rpm * (2.0 * 3.14159265 / 60.0)

        self.status_pub = self.create_publisher(String, 'nammy/motors/status', 10)
        self.limits_pub = self.create_publisher(Float32MultiArray, 'nammy/motors/limits', 10)

        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info("==================================================================")
        self.get_logger().info("NAMMY MOTOR MANAGER INITIALIZED (SmartElex 30S & AT120 DC Motors)")
        self.get_logger().info(f"Rated Speed: {self.max_rpm:.1f} RPM ({self.max_rad_s:.2f} rad/s) | Gearbox: {self.gear_ratio:.1f}:1")
        self.get_logger().info(f"Driver Rating: 30A Continuous / 85A Peak @ 24V DC")
        self.get_logger().info("==================================================================")

    def publish_status(self):
        msg = String()
        msg.data = f"SmartElex 30S Dual: ACTIVE | MaxRadS: {self.max_rad_s:.2f} | 20kHz Silent PWM"
        self.status_pub.publish(msg)

        limits_msg = Float32MultiArray()
        limits_msg.data = [float(self.max_rad_s), float(self.max_rpm), float(self.gear_ratio)]
        self.limits_pub.publish(limits_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
