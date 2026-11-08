#!/usr/bin/env python3
"""
Backward-compatibility forwarder for Nammy Autonomous SLAM Mapping.
Forwards all parameters directly to the dedicated 'nammy_mapping' package.

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping_pkg = get_package_share_directory('nammy_mapping')
    launch_path = os.path.join(mapping_pkg, 'launch', 'mapping.launch.py')

    default_serial = '/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0'

    declared_args = [
        DeclareLaunchArgument('simulate_hardware', default_value='false', description='Simulate hardware fallback'),
        DeclareLaunchArgument('simulate_lidar', default_value='false', description='Simulate lidar fallback'),
        DeclareLaunchArgument('serial_port', default_value=default_serial, description='Serial port for ESP32'),
        DeclareLaunchArgument('i2c_bus', default_value='7', description='I2C bus for BNO055 IMU'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='Enable robot_localization EKF'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('teleop', default_value='true', description='Launch keyboard teleop xterm'),
        DeclareLaunchArgument('launch_camera', default_value='false', description='Launch C270 camera')
    ]

    include_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'simulate_hardware': LaunchConfiguration('simulate_hardware'),
            'simulate_lidar': LaunchConfiguration('simulate_lidar'),
            'serial_port': LaunchConfiguration('serial_port'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'use_ekf': LaunchConfiguration('use_ekf'),
            'rviz': LaunchConfiguration('rviz'),
            'teleop': LaunchConfiguration('teleop'),
            'launch_camera': LaunchConfiguration('launch_camera')
        }.items()
    )

    return LaunchDescription(declared_args + [include_action])
