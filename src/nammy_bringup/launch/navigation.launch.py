#!/usr/bin/env python3
"""
Nav2 Autonomous Navigation Launch File for Nammy Robot Platform.
Integrates complete robot bringup with YDLIDAR G2, BNO055 IMU, C270 camera,
robot_localization EKF sensor fusion, Map Server, AMCL localization,
and full Nav2 navigation stack (DWB local planner, Navfn global planner,
costmaps, recovery behaviors, and lifecycle manager).

Directly derived from proven hover_ws production navigation architecture.

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
    nav2_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(os.path.expanduser('~'), 'Documents', 'nammy_ws', 'maps', 'room1_2Dmap.yaml')
    default_serial = '/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0'

    ekf_params_file = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    nav2_params_file = os.path.join(bringup_pkg, 'config', 'nav2_params.yaml')
    rviz_config_file = os.path.join(desc_pkg, 'rviz', 'navigation.rviz')

    # Arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to 2D occupancy grid map YAML file'
    )
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
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=default_serial,
        description='Serial port for ESP32 hardware bridge'
    )
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number for BNO055 IMU on Jetson 40-pin header'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with Nav2 autonomous navigation visualizer'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes'
    )

    # 1. Base Robot Bringup (ESP32 hardware, BNO055 IMU, C270 Camera, 4x US-100 Sonars, YDLIDAR G2)
    # Odometry TF is handled exclusively by EKF
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'simulate_hardware': LaunchConfiguration('simulate_hardware'),
            'simulate_lidar': LaunchConfiguration('simulate_lidar'),
            'serial_port': LaunchConfiguration('serial_port'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'publish_odom_tf': 'false',
            'launch_rviz': 'false',
            'launch_lidar': 'true',
            'launch_camera': 'true',
            'launch_sonar': 'true',
        }.items()
    )

    # 2. Extended Kalman Filter (robot_localization EKF)
    ekf_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[
                    ekf_params_file,
                    {'use_sim_time': False}
                ]
            )
        ]
    )

    # 3. Nav2 Map Server
    map_server_node = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    {'yaml_filename': LaunchConfiguration('map')},
                    {'use_sim_time': False}
                ]
            )
        ]
    )

    # 4. AMCL Probabilistic Localization
    amcl_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[
                    nav2_params_file,
                    {'use_sim_time': False}
                ]
            )
        ]
    )

    # 5. Localization Lifecycle Manager
    lifecycle_localization = TimerAction(
        period=7.5,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'autostart': LaunchConfiguration('autostart')},
                    {'node_names': ['map_server', 'amcl']}
                ]
            )
        ]
    )

    # 6. Full Nav2 Navigation Stack (DWB Local Planner, Navfn Global Planner, Costmaps, BT Navigator)
    nav2_stack = TimerAction(
        period=9.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file,
                    'autostart': LaunchConfiguration('autostart'),
                }.items()
            )
        ]
    )

    # 7. RViz2 Visualizer with Navigation 2 UI
    rviz_node = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_navigation',
                output='screen',
                arguments=['-d', rviz_config_file],
                condition=IfCondition(LaunchConfiguration('rviz'))
            )
        ]
    )

    return LaunchDescription([
        map_arg,
        sim_hw_arg,
        sim_lidar_arg,
        serial_port_arg,
        i2c_bus_arg,
        launch_rviz_arg,
        autostart_arg,
        robot_bringup,
        ekf_node,
        map_server_node,
        amcl_node,
        lifecycle_localization,
        nav2_stack,
        rviz_node
    ])
