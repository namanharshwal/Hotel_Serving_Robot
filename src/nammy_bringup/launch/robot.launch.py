#!/usr/bin/env python3
"""
Master Robot Bringup Launch File for Nammy Robot Platform.
Coordinates and launches all component packages:
  - nammy_description (URDF, Robot State Publisher)
  - nammy_base        (ESP32-S3 Hardware Bridge & Odometry)
  - nammy_motors      (SmartElex 30S & AT120 Motor Manager)
  - nammy_imu         (Bosch BNO055 9-DOF IMU Driver)
  - nammy_power       (24V LiFePO4 Battery Monitor & Safety Alerts)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    desc_pkg = get_package_share_directory('nammy_description')
    bringup_pkg = get_package_share_directory('nammy_bringup')

    # Process URDF Xacro
    xacro_file = os.path.join(desc_pkg, 'urdf', 'nammy_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Centralized Parameter File
    params_file = os.path.join(bringup_pkg, 'config', 'robot_params.yaml')

    sim_arg = DeclareLaunchArgument(
        'simulate_hardware',
        default_value='false',
        description='Run controller in software simulation fallback without hardware'
    )

    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/nammy_esp32',
        description='Serial port device path for ESP32-S3'
    )

    imu_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number for BNO055 on Jetson 40-pin header'
    )

    # 1. Robot State Publisher (3D URDF)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 2. ESP32-S3 Mediator Hardware Bridge (nammy_base)
    base_bridge_node = Node(
        package='nammy_base',
        executable='esp32_hardware_bridge',
        name='esp32_hardware_bridge',
        output='screen',
        parameters=[
            params_file,
            {
                'simulate_hardware': LaunchConfiguration('simulate_hardware'),
                'serial_port': LaunchConfiguration('serial_port'),
            }
        ]
    )

    # 3. Motor Manager (nammy_motors)
    motor_manager_node = Node(
        package='nammy_motors',
        executable='motor_manager',
        name='motor_manager',
        output='screen',
        parameters=[params_file]
    )

    # 4. Bosch BNO055 9-DOF IMU (nammy_imu)
    imu_node = Node(
        package='nammy_imu',
        executable='bno055_node',
        name='bno055_node',
        output='screen',
        parameters=[
            params_file,
            {
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'simulate_hardware': LaunchConfiguration('simulate_hardware'),
            }
        ]
    )

    # 5. 24V LiFePO4 Battery Monitor (nammy_power)
    power_node = Node(
        package='nammy_power',
        executable='battery_monitor_node',
        name='battery_monitor_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        sim_arg,
        port_arg,
        imu_bus_arg,
        robot_state_pub_node,
        base_bridge_node,
        motor_manager_node,
        imu_node,
        power_node
    ])
