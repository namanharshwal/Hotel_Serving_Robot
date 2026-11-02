#!/usr/bin/env python3
"""
Touch HMI Dashboard Launch File for 10.1" Waveshare Capacitive Display.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nammy_display',
            executable='touch_hmi_app',
            name='nammy_touch_hmi',
            output='screen'
        )
    ])
