#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - Autonomous SLAM Mapping Launch File (nammy_mapping)
=============================================================================
Description:
  Dedicated autonomous mapping and SLAM package for the Nammy Robot Platform.
  Coordinates and orchestrates:
    - Base robot hardware bringup (nammy_bringup: ESP32, motors, BNO055 IMU, YDLIDAR G2, Sonars)
    - Robot localization EKF sensor fusion (wheel odometry + IMU)
    - SLAM Toolbox (online asynchronous graph-based mapping)
    - RViz2 Mapping Visualization (15 FPS, robust memory profile)
    - Interactive Keyboard Teleoperation in an xterm window on DISPLAY

Maintainer: Naman Harshwal <namanharshwal@gmail.com>
Copyright (c) 2026 Naman Harshwal. All rights reserved.
=============================================================================
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
    mapping_pkg = get_package_share_directory('nammy_mapping')
    bringup_pkg = get_package_share_directory('nammy_bringup')

    slam_params_file = os.path.join(mapping_pkg, 'config', 'slam_toolbox_params.yaml')
    ekf_params_file = os.path.join(bringup_pkg, 'config', 'ekf.yaml')
    rviz_config_file = os.path.join(mapping_pkg, 'rviz', 'mapping.rviz')

    default_serial = '/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0'

    # Arguments
    sim_hw_arg = DeclareLaunchArgument(
        'simulate_hardware',
        default_value='false',
        description='Simulate ESP32 motor controller fallback'
    )
    sim_lidar_arg = DeclareLaunchArgument(
        'simulate_lidar',
        default_value='false',
        description='Simulate 360-degree laser scan fallback if physical lidar is not connected'
    )
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
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='false',
        description='Launch Logitech C270 camera driver'
    )

    # When EKF is enabled, suppress wheel odometry TF from ESP32 bridge so EKF publishes odom->base_footprint TF cleanly
    publish_odom_tf_expr = PythonExpression([
        "'false' if '", LaunchConfiguration('use_ekf'), "' == 'true' else 'true'"
    ])

    # 1. Base Robot Bringup (ESP32 hardware, SmartElex motors, BNO055 IMU, Sonars, YDLIDAR G2)
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
            'launch_camera': LaunchConfiguration('launch_camera'),
            'launch_sonar': 'true',
            'teleop': 'false',
        }.items()
    )

    # 2. Extended Kalman Filter (robot_localization EKF)
    ekf_node = TimerAction(
        period=2.0,
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

    # 3. Interactive Keyboard Teleoperation Window (opens in xterm window on DISPLAY)
    teleop_process = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-title', 'Nammy Robot SLAM Teleop Control',
                    '-geometry', '88x26+40+40',
                    '-fa', 'Monospace',
                    '-fs', '11',
                    '-bg', '#1c1c1c',
                    '-fg', '#00ff88',
                    '-e', 'bash', '-c',
                    'source /opt/ros/humble/setup.bash && [ -f "$HOME/Documents/hover_ws/install/setup.bash" ] && source "$HOME/Documents/hover_ws/install/setup.bash"; [ -f "$HOME/Documents/nammy_ws/install/setup.bash" ] && source "$HOME/Documents/nammy_ws/install/setup.bash"; export ROS_DOMAIN_ID=0 && ros2 run nammy_teleop teleop_keyboard; exec bash'
                ],
                condition=IfCondition(LaunchConfiguration('teleop')),
                output='screen'
            )
        ]
    )

    # 4. SLAM Toolbox Online Async Node (delayed 7.0s for LiDAR spin-up, TF, EKF, and /scan stabilization)
    slam_toolbox_node = TimerAction(
        period=7.0,
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

    # 5. RViz2 configured with Map, Scan, RobotModel, TF (delayed 10.0s)
    rviz_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_mapping',
                output='screen',
                arguments=['-d', rviz_config_file],
                condition=IfCondition(LaunchConfiguration('rviz'))
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
        launch_camera_arg,
        robot_bringup,
        ekf_node,
        teleop_process,
        slam_toolbox_node,
        rviz_node
    ])
