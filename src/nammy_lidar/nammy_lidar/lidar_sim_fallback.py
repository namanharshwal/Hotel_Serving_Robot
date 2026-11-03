#!/usr/bin/env python3
"""
YDLIDAR G2 Simulation Fallback Node for Nammy Robot.
Generates synthetic 360-degree laser scans when physical LiDAR is not connected.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSimFallback(Node):
    def __init__(self):
        super().__init__('lidar_sim_fallback')

        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('scan_frequency', 10.0)  # 10 Hz
        self.declare_parameter('samples', 360)          # 360 points (1 deg resolution)
        self.declare_parameter('sim_room_radius', 3.5)  # 3.5m simulated room boundary

        self.frame_id = self.get_parameter('frame_id').value
        self.freq = self.get_parameter('scan_frequency').value
        self.samples = self.get_parameter('samples').value
        self.room_r = self.get_parameter('sim_room_radius').value

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(1.0 / self.freq, self.publish_scan)
        self.get_logger().info("YDLIDAR G2 Simulation Fallback active: publishing /scan at 10 Hz")

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (2.0 * math.pi) / self.samples
        msg.time_increment = (1.0 / self.freq) / self.samples
        msg.scan_time = 1.0 / self.freq
        msg.range_min = 0.12
        msg.range_max = 12.0

        # Create simulated room boundary with realistic noise
        ranges = []
        for i in range(self.samples):
            theta = msg.angle_min + i * msg.angle_increment
            # Circular room with occasional posts/walls
            dist = self.room_r + 0.15 * math.sin(4.0 * theta)
            ranges.append(float(dist))

        msg.ranges = ranges
        msg.intensities = [100.0] * self.samples
        self.scan_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarSimFallback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
