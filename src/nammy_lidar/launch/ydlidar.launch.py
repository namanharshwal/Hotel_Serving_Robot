#!/usr/bin/env python3
"""
YDLIDAR G2 Driver Launch File for Nammy Robot.
Supports both physical hardware launch (/dev/ydlidar) and automatic simulation fallback.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('nammy_lidar')
    default_params = os.path.join(pkg_share, 'config', 'ydlidar_g2_params.yaml')

    sim_arg = DeclareLaunchArgument(
        'simulate_lidar',
        default_value='false',
        description='Force simulation fallback without hardware'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to YDLIDAR G2 YAML parameters'
    )

    # Physical hardware node (from system installed ydlidar_ros2_driver)
    real_lidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        condition=UnlessCondition(LaunchConfiguration('simulate_lidar'))
    )

    # Simulation fallback node
    sim_lidar_node = Node(
        package='nammy_lidar',
        executable='lidar_sim_fallback',
        name='lidar_sim_fallback',
        output='screen',
        condition=IfCondition(LaunchConfiguration('simulate_lidar'))
    )

    return LaunchDescription([
        sim_arg,
        params_arg,
        real_lidar_node,
        sim_lidar_node
    ])
