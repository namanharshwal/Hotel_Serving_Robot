#!/usr/bin/env python3
"""
Launch file for Logitech C270 Webcam Node on Nammy Robot Platform.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nammy_camera')
    default_params = os.path.join(pkg_share, 'config', 'c270_params.yaml')

    params_arg = DeclareLaunchArgument(
        'camera_params',
        default_value=default_params,
        description='Path to Logitech C270 YAML parameter file'
    )

    c270_node = Node(
        package='nammy_camera',
        executable='c270_camera_node',
        name='c270_camera_node',
        output='screen',
        parameters=[LaunchConfiguration('camera_params')]
    )

    return LaunchDescription([
        params_arg,
        c270_node
    ])
