#!/usr/bin/env python3
"""
Sonar Fusion Node for Nammy Robot.
Fuses 4 US-100 ultrasonic sensor cones into a PointCloud2 obstacle map for Nav2 costmaps,
enabling detection of glass walls, mirrors, low furniture, and small obstacles missed by 2D LiDAR.
"""

import math
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2, PointField
from std_msgs.msg import Header, Bool

class SonarFusionNode(Node):
    def __init__(self):
        super().__init__('sonar_fusion_node')

        # Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('critical_stop_distance', 0.15)  # 15 cm
        self.declare_parameter('max_detection_range', 3.50)     # 3.5 m
        self.declare_parameter('points_per_cone', 5)            # Number of points representing ultrasonic beam width

        self.base_frame = self.get_parameter('base_frame').value
        self.critical_dist = self.get_parameter('critical_stop_distance').value
        self.max_range = self.get_parameter('max_detection_range').value
        self.num_pts = self.get_parameter('points_per_cone').value

        # Latest readings (distance in meters)
        self.sonar_ranges = {
            'front': None,
            'left': None,
            'right': None,
            'rear': None
        }

        # Sensor positions relative to base_link (x, y, z, yaw)
        # Matches nammy_robot.urdf.xacro exactly!
        self.sensor_transforms = {
            'front': (0.235,  0.000, 0.060, 0.0),
            'left':  (0.000,  0.165, 0.060, math.pi / 2.0),
            'right': (0.000, -0.165, 0.060, -math.pi / 2.0),
            'rear':  (-0.235, 0.000, 0.060, math.pi)
        }

        # Subscribers
        self.sub_front = self.create_subscription(Range, 'sonar/front', lambda m: self.range_cb('front', m), 10)
        self.sub_left  = self.create_subscription(Range, 'sonar/left',  lambda m: self.range_cb('left', m), 10)
        self.sub_right = self.create_subscription(Range, 'sonar/right', lambda m: self.range_cb('right', m), 10)
        self.sub_rear  = self.create_subscription(Range, 'sonar/rear',  lambda m: self.range_cb('rear', m), 10)

        # Publishers
        self.cloud_pub = self.create_publisher(PointCloud2, 'sonar/obstacle_cloud', 10)
        self.safety_pub = self.create_publisher(Bool, 'nammy/sonar_safety_stop', 10)

        # Periodic fusion loop at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_fused_cloud)
        self.get_logger().info("Sonar Fusion Node initialized: 4 US-100 sensors -> PointCloud2 Costmap")

    def range_cb(self, sensor_name: str, msg: Range):
        self.sonar_ranges[sensor_name] = msg.range

    def publish_fused_cloud(self):
        points = []
        critical_stop = False

        for name, r in self.sonar_ranges.items():
            if r is None or r <= 0.02 or r >= self.max_range:
                continue

            if r < self.critical_dist:
                critical_stop = True

            # Sensor pose in base_link
            sx, sy, sz, syaw = self.sensor_transforms[name]
            fov = 0.2618  # 15 degrees cone for US-100

            # Generate arc of points across the beam width
            half_fov = fov / 2.0
            angles = [syaw - half_fov + (i / max(1, self.num_pts - 1)) * fov for i in range(self.num_pts)]

            for ang in angles:
                px = sx + r * math.cos(ang)
                py = sy + r * math.sin(ang)
                pz = sz
                points.append((px, py, pz))

        # Publish safety reflex
        safety_msg = Bool()
        safety_msg.data = critical_stop
        self.safety_pub.publish(safety_msg)

        if not points:
            return

        # Pack PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = bytearray()
        for pt in points:
            cloud_data.extend(struct.pack('<fff', pt[0], pt[1], pt[2]))

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = 12 * len(points)
        cloud_msg.data = bytes(cloud_data)
        cloud_msg.is_dense = True

        self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SonarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
