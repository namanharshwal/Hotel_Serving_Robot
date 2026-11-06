#!/usr/bin/env python3
"""
Master Robot Bringup Launch File for Nammy Robot Platform.
Coordinates and launches all component packages:
  - nammy_description (URDF, Robot State Publisher, 4x Sonars, YDLIDAR G2)
  - nammy_base        (ESP32-S3 Hardware Bridge, Odometry & Range Publisher)
  - nammy_motors      (SmartElex 30S & AT120 Motor Manager)
  - nammy_imu         (Bosch BNO055 9-DOF IMU Driver)
  - nammy_power       (24V LiFePO4 Battery Monitor & Safety Alerts)
  - nammy_sonar       (4-Channel US-100 Sonar Fusion & PointCloud2 Costmap)
  - nammy_lidar       (YDLIDAR G2 360-Degree 2D Laser Scanner)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    desc_pkg = get_package_share_directory('nammy_description')
    bringup_pkg = get_package_share_directory('nammy_bringup')
    lidar_pkg = get_package_share_directory('nammy_lidar')
    sonar_pkg = get_package_share_directory('nammy_sonar')
    camera_pkg = get_package_share_directory('nammy_camera')

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

    default_serial = '/dev/nammy_esp32' if os.path.exists('/dev/nammy_esp32') else '/dev/ttyACM0'
    port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=default_serial,
        description='Serial port device path for ESP32-S3'
    )

    imu_bus_arg = DeclareLaunchArgument(
        'i2c_bus',
        default_value='7',
        description='I2C bus number for BNO055 on Jetson 40-pin header'
    )

    lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Launch YDLIDAR G2 driver node'
    )

    sim_lidar_arg = DeclareLaunchArgument(
        'simulate_lidar',
        default_value='false',
        description='Run simulated laser scan if hardware is not plugged in'
    )

    sonar_arg = DeclareLaunchArgument(
        'launch_sonar',
        default_value='true',
        description='Launch 4x US-100 Sonar Fusion & Costmap PointCloud2'
    )

    camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch Logitech C270 HD camera vision publisher'
    )

    rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz2 3D visualizer window'
    )

    publish_odom_tf_arg = DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='true',
        description='Publish odom->base_footprint TF from base bridge (set false when EKF is active)'
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
                'publish_tf': LaunchConfiguration('publish_odom_tf'),
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

    # 6. 4x US-100 Sonar Fusion Node (nammy_sonar)
    sonar_node = Node(
        package='nammy_sonar',
        executable='sonar_fusion_node',
        name='sonar_fusion_node',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('launch_sonar'))
    )

    # 7. YDLIDAR G2 Launch (nammy_lidar)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'ydlidar.launch.py')
        ),
        launch_arguments={
            'simulate_lidar': LaunchConfiguration('simulate_lidar')
        }.items(),
        condition=IfCondition(LaunchConfiguration('launch_lidar'))
    )

    # 8. Logitech C270 HD Webcam Vision Node (nammy_camera)
    camera_node = Node(
        package='nammy_camera',
        executable='c270_camera_node',
        name='c270_camera_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_camera'))
    )

    # 9. RViz2 3D Robot Visualizer Window
    rviz_config_file = os.path.join(desc_pkg, 'rviz', 'view_robot.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        sim_arg,
        port_arg,
        imu_bus_arg,
        lidar_arg,
        sim_lidar_arg,
        sonar_arg,
        camera_arg,
        rviz_arg,
        publish_odom_tf_arg,
        robot_state_pub_node,
        base_bridge_node,
        motor_manager_node,
        imu_node,
        power_node,
        sonar_node,
        lidar_launch,
        camera_node,
        rviz_node
    ])
