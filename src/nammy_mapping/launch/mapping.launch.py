#!/usr/bin/env python3
"""
=============================================================================
Nammy Robot - Autonomous SLAM Mapping Launch File (nammy_mapping)
=============================================================================
Description:
  Unified Autonomous SLAM Mapping package for Nammy Robot Platform.
  Matches the exact architectural model of Autonomous Navigation and Manual Mapping:
    - Base robot hardware bringup (ESP32-S3, SmartElex motors, BNO055 IMU, 4x US-100 Sonars, YDLIDAR G2)
    - Heading hold disabled during mapping so Nav2 local controller has full rotational authority
    - Robot localization EKF sensor fusion (wheel odometry + IMU)
    - SLAM Toolbox (online asynchronous graph-based mapping) providing dynamic /map & map->odom TF
    - Full Nav2 Navigation Stack (DWB Local Planner, Navfn Planner, Costmaps, BT Navigator)
    - RViz2 Navigation Visualization with full Nav2 panel, Goal Tool, Costmaps, Footprints, Plans, RobotModel
    - Interactive RViz Waypoint Collector for autonomous exploration and delivery
    - Interactive Keyboard Teleoperation in an xterm window for manual drive override

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
    nav2_pkg = get_package_share_directory('nav2_bringup')

    slam_params_file = os.path.join(mapping_pkg, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(mapping_pkg, 'config', 'nav2_params.yaml')
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
        description='Launch RViz2 with complete navigation and mapping interface'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes'
    )
    interactive_waypoints_arg = DeclareLaunchArgument(
        'interactive_waypoints',
        default_value='true',
        description='Launch interactive xterm terminal for RViz Publish Point waypoint collection and exploration missions'
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
    # enable_imu_heading_hold: false so Nav2 controller has full authority over steering
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
            'enable_imu_heading_hold': 'false',
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

    # 3. SLAM Toolbox Online Async Node (provides live /map and map->odom TF)
    slam_toolbox_node = TimerAction(
        period=5.0,
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

    # 4. Full Nav2 Navigation Stack (DWB Local Planner, Navfn Planner, Costmaps, BT Navigator)
    nav2_stack = TimerAction(
        period=8.5,
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

    # 5. RViz2 configured with exact Navigation and Mapping Model (delayed 11.5s)
    rviz_node = TimerAction(
        period=11.5,
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

    # 6. Interactive RViz Waypoint Collector & Autonomous Exploration (delayed 12.5s)
    waypoint_collector_process = TimerAction(
        period=12.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-title', 'Nammy SLAM Waypoint Mission Controller',
                    '-geometry', '88x24+40+40',
                    '-fa', 'Monospace',
                    '-fs', '11',
                    '-bg', '#0f172a',
                    '-fg', '#38bdf8',
                    '-e', 'bash', '-c',
                    'export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash && [ -f "$HOME/Documents/hover_ws/install/setup.bash" ] && source "$HOME/Documents/hover_ws/install/setup.bash"; [ -f "$HOME/Documents/nammy_ws/install/setup.bash" ] && source "$HOME/Documents/nammy_ws/install/setup.bash"; ros2 run nammy_mapping waypoint_collector.py; exec bash'
                ],
                condition=IfCondition(LaunchConfiguration('interactive_waypoints')),
                output='screen'
            )
        ]
    )

    # 7. Interactive Keyboard Teleoperation Window for manual override (delayed 13.5s)
    teleop_process = TimerAction(
        period=13.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'xterm',
                    '-title', 'Nammy SLAM Manual Teleop Controller',
                    '-geometry', '85x24+600+40',
                    '-fa', 'Monospace',
                    '-fs', '11',
                    '-bg', '#111827',
                    '-fg', '#10b981',
                    '-e', 'bash', '-c',
                    'export ROS_DOMAIN_ID=0 && source /opt/ros/humble/setup.bash && [ -f "$HOME/Documents/hover_ws/install/setup.bash" ] && source "$HOME/Documents/hover_ws/install/setup.bash"; [ -f "$HOME/Documents/nammy_ws/install/setup.bash" ] && source "$HOME/Documents/nammy_ws/install/setup.bash"; ros2 run nammy_teleop teleop_keyboard; exec bash'
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
        autostart_arg,
        interactive_waypoints_arg,
        teleop_arg,
        launch_camera_arg,
        robot_bringup,
        ekf_node,
        slam_toolbox_node,
        nav2_stack,
        rviz_node,
        waypoint_collector_process,
        teleop_process
    ])
