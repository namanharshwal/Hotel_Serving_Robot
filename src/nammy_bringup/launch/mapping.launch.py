#!/usr/bin/env python3
"""
SLAM Mapping Launch File for Nammy Robot Platform with Interactive Teleoperation.
Integrates complete robot bringup with YDLIDAR G2, BNO055 IMU, C270 camera,
robot_localization EKF sensor fusion, SLAM Toolbox (online async mapping),
and interactive terminal teleoperation.

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('nammy_bringup')
    desc_pkg = get_package_share_directory('nammy_description')

    slam_params_file = os.path.join(bringup_pkg, 'config', 'slam_toolbox_params.yaml')
    ekf_params_file = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    rviz_config_file = os.path.join(desc_pkg, 'rviz', 'mapping.rviz')

    # Arguments
    sim_hw_arg = DeclareLaunchArgument(
        'simulate_hardware',
        default_value='false',
        description='Simulate ESP32 motor controller fallback'
    )
    # Default to physical LiDAR hardware (never launch simulation fallback unless explicitly requested)
    sim_lidar_arg = DeclareLaunchArgument(
        'simulate_lidar',
        default_value='false',
        description='Simulate 360-degree laser scan fallback if physical lidar is not connected'
    )
    default_serial = '/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0'
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=default_serial,
        description='Serial port for ESP32 hardware bridge'
    )
    i2c_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number for BNO055 IMU on Jetson 40-pin header (Pin 27=SDA, Pin 28=SCL -> /dev/i2c-7)'
    )
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Enable robot_localization EKF sensor fusion for wheel odometry and BNO055 IMU'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 with SLAM mapping configuration'
    )
    teleop_arg = DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='Launch interactive keyboard teleoperation in an xterm window on DISPLAY'
    )

    # When EKF is enabled, suppress wheel odometry TF from ESP32 bridge so EKF publishes odom->base_footprint TF cleanly
    publish_odom_tf_expr = PythonExpression([
        "'false' if '", LaunchConfiguration('use_ekf'), "' == 'true' else 'true'"
    ])

    # 1. Base Robot Bringup (ESP32 hardware, SmartElex motors, BNO055 IMU, C270 Camera, 4x US-100 Sonars, YDLIDAR G2)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'simulate_hardware': LaunchConfiguration('simulate_hardware'),
            'simulate_lidar': LaunchConfiguration('simulate_lidar'),
            'serial_port': LaunchConfiguration('serial_port'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'publish_odom_tf': publish_odom_tf_expr,
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
                ],
                condition=IfCondition(LaunchConfiguration('use_ekf'))
            )
        ]
    )

    # 3. SLAM Toolbox Online Async Node (delayed 4.5s for TF, EKF, and /scan stabilization, matching hover_ws)
    slam_toolbox_node = TimerAction(
        period=4.5,
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

    # 4. RViz2 configured with Map, Scan, RobotModel, TF, and Camera
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # 5. Interactive Keyboard Teleoperation Window (opens in xterm window on DISPLAY=:1)
    teleop_process = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-title', 'Nammy Robot Teleop Control',
                    '-geometry', '88x26+40+40',
                    '-fa', 'Monospace',
                    '-fs', '11',
                    '-bg', '#1c1c1c',
                    '-fg', '#00ff88',
                    '-e', 'bash', '-c',
                    'source /opt/ros/humble/setup.bash && source /home/comppoint/Documents/nammy_ws/install/setup.bash && export ROS_DOMAIN_ID=0 && ros2 run nammy_teleop teleop_keyboard; exec bash'
                ],
                condition=IfCondition(LaunchConfiguration('teleop')),
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        sim_hw_arg,
        sim_lidar_arg,
        serial_port_arg,
        i2c_bus_arg,
        use_ekf_arg,
        launch_rviz_arg,
        teleop_arg,
        robot_bringup,
        ekf_node,
        slam_toolbox_node,
        rviz_node,
        teleop_process
    ])
