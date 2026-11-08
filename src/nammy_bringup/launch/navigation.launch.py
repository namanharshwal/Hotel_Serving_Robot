#!/usr/bin/env python3
"""
Backward-compatibility forwarder for Nammy Autonomous Navigation.
Forwards all parameters directly to the dedicated 'nammy_navigation' package.

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
    nav_pkg = get_package_share_directory('nammy_navigation')
    launch_path = os.path.join(nav_pkg, 'launch', 'navigation.launch.py')

    default_map = os.path.join(os.path.expanduser('~'), 'Documents', 'nammy_ws', 'maps', 'raj_sir_home_lab_room.yaml')
    default_serial = '/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0'

    declared_args = [
        DeclareLaunchArgument('map', default_value=default_map, description='Occupancy grid map YAML file'),
        DeclareLaunchArgument('simulate_hardware', default_value='false', description='Simulate hardware fallback'),
        DeclareLaunchArgument('simulate_lidar', default_value='false', description='Simulate lidar fallback'),
        DeclareLaunchArgument('serial_port', default_value=default_serial, description='Serial port for ESP32'),
        DeclareLaunchArgument('i2c_bus', default_value='7', description='I2C bus for BNO055 IMU'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('autostart', default_value='true', description='Autostart lifecycle nodes'),
        DeclareLaunchArgument('interactive_waypoints', default_value='true', description='Interactive waypoint terminal')
    ]

    include_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'simulate_hardware': LaunchConfiguration('simulate_hardware'),
            'simulate_lidar': LaunchConfiguration('simulate_lidar'),
            'serial_port': LaunchConfiguration('serial_port'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'rviz': LaunchConfiguration('rviz'),
            'autostart': LaunchConfiguration('autostart'),
            'interactive_waypoints': LaunchConfiguration('interactive_waypoints'),
        }.items()
    )

    return LaunchDescription(declared_args + [include_action])
