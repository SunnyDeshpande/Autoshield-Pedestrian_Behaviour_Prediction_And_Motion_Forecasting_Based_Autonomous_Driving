#!/usr/bin/env python3
# launch/high_level_command_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='',
        description='Vehicle identifier (e.g., e2, e4)'
    )

    # High-level command node
    command_node = Node(
        package='autoshield_full',
        executable='high_level_command',
        name='high_level_command',
        output='screen',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
        }],
    )

    return LaunchDescription([
        vehicle_name_arg,
        command_node,
    ])
