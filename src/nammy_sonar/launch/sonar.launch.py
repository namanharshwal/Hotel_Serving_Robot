#!/usr/bin/env python3
"""
Sonar Fusion Launch File for Nammy Robot.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nammy_sonar',
            executable='sonar_fusion_node',
            name='sonar_fusion_node',
            output='screen',
            parameters=[{
                'base_frame': 'base_link',
                'critical_stop_distance': 0.15,
                'max_detection_range': 3.50,
                'points_per_cone': 5
            }]
        )
    ])
