#!/usr/bin/env python3
"""
=============================================================================
Hotel Serving Robot (Nammy Robot) - Physical Manual Control (Zero Simulation)
=============================================================================
Description:
  Dedicated launch file for manual robot movement with live camera feed.
  
  Launches:
    - ESP32-S3 Hardware Bridge (/dev/nammy_esp32)
    - Bosch BNO055 9-DOF IMU (/dev/i2c-7) for active closed-loop heading hold
    - Motor Manager (SmartElex 30S & AT120 planetary gear motors)
    - Battery Monitor (24V LiFePO4 protection)
    - US-100 Sonar Safety Reflex
    - Logitech C270 HD Camera Publisher (/camera/image_raw)
    - Live PyQt5 Camera Viewer HUD Window (positioned side-by-side on DISPLAY)
    - Interactive Keyboard Teleoperation (xterm on DISPLAY)

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
=============================================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('nammy_bringup')
    robot_launch_path = os.path.join(bringup_pkg, 'launch', 'robot.launch.py')

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0',
        description='Serial port for ESP32-S3 hardware'
    )

    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number for BNO055 IMU on Jetson 40-pin header'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz 3D visualizer (default false for pure manual teleop)'
    )

    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='false',
        description='Launch YDLIDAR G2 driver (default false for pure manual movement)'
    )

    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch Logitech C270 camera driver and live vision viewer'
    )

    robot_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path),
        launch_arguments={
            'simulate_hardware': 'false',
            'simulate_lidar': 'false',
            'serial_port': LaunchConfiguration('serial_port'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'launch_lidar': LaunchConfiguration('launch_lidar'),
            'launch_camera': LaunchConfiguration('launch_camera'),
            'launch_sonar': 'true',
            'publish_odom_tf': 'true',
            'teleop': 'true'
        }.items()
    )

    # Launch live camera viewer window alongside teleop terminal (delayed 2.5s for camera device init)
    camera_viewer_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='nammy_camera',
                executable='camera_viewer',
                name='camera_viewer',
                output='screen',
                condition=IfCondition(LaunchConfiguration('launch_camera'))
            )
        ]
    )

    return LaunchDescription([
        serial_port_arg,
        i2c_bus_arg,
        launch_rviz_arg,
        launch_lidar_arg,
        launch_camera_arg,
        robot_base,
        camera_viewer_node
    ])
