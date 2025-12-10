#!/usr/bin/env python3
# launch/autoshield_full_launch.py
"""
Full AutoShield system launch file
Launches both lidar preprocessing and straight path control
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='',
        description='Vehicle identifier (e.g., e2, e4)'
    )

    desired_speed_arg = DeclareLaunchArgument(
        'desired_speed',
        default_value='2.0',
        description='Desired vehicle speed in m/s (max: 5.0)'
    )

    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable lidar preprocessing node'
    )

    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='true',
        description='Enable straight path control node'
    )

    # Get paths to config files
    lidar_config = PathJoinSubstitution([
        FindPackageShare('autoshield_full'),
        'config',
        'lidar_params.yaml'
    ])

    straight_path_config = PathJoinSubstitution([
        FindPackageShare('autoshield_full'),
        'config',
        'straight_path_params.yaml'
    ])

    # Lidar preprocessing node
    lidar_node = Node(
        package='lidar_person_detection',
        executable='lidar_preprocessor',
        name='lidar_preprocessor',
        output='screen',
        parameters=[lidar_config],
        condition=IfCondition(LaunchConfiguration('enable_lidar'))
    )

    # Straight path controller node
    straight_path_node = Node(
        package='autoshield_full',
        executable='straight_path',
        name='autoshield_straight_path',
        output='screen',
        parameters=[
            straight_path_config,
            {
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'desired_speed': LaunchConfiguration('desired_speed'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_control'))
    )

    return LaunchDescription([
        vehicle_name_arg,
        desired_speed_arg,
        enable_lidar_arg,
        enable_control_arg,
        lidar_node,
        straight_path_node,
    ])
