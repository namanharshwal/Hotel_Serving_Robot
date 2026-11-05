#!/usr/bin/env python3
"""
SLAM Mapping Launch File for Nammy Robot Platform.
Integrates complete robot bringup with YDLIDAR G2 and SLAM Toolbox (online async mapping),
with exact parameters and sequencing derived from hover_ws.

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
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
    desc_pkg = get_package_share_directory('nammy_description')

    slam_params_file = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    rviz_config_file = os.path.join(desc_pkg, 'rviz', 'mapping.rviz')

    # Arguments
    sim_hw_arg = DeclareLaunchArgument(
        'simulate_hardware',
        default_value='false',
        description='Simulate ESP32 motor controller fallback'
    )
    sim_lidar_arg = DeclareLaunchArgument(
        'simulate_lidar',
        default_value='false',
        description='Simulate 360-degree laser scan fallback'
    )
    default_serial = '/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0'
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=default_serial,
        description='Serial port for ESP32 hardware bridge'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with SLAM mapping configuration'
    )

    # 1. Include Base Robot Bringup (without starting default RViz; mapping RViz is started below)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'simulate_hardware': LaunchConfiguration('simulate_hardware'),
            'simulate_lidar': LaunchConfiguration('simulate_lidar'),
            'serial_port': LaunchConfiguration('serial_port'),
            'launch_rviz': 'false',
            'launch_lidar': 'true',
        }.items()
    )

    # 2. SLAM Toolbox Online Async Node (delayed 4.0s for TF and /scan stabilization, matching hover_ws)
    slam_toolbox_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_params_file,
                    {'use_sim_time': False}
                ],
                remappings=[('/scan', '/scan'), ('/map', '/map')]
            )
        ]
    )

    # 3. RViz2 configured with Map, Scan, RobotModel, TF, and Odometry
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        sim_hw_arg,
        sim_lidar_arg,
        serial_port_arg,
        launch_rviz_arg,
        robot_bringup,
        slam_toolbox_node,
        rviz_node
    ])
