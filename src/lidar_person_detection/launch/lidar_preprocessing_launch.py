# launch/lidar_preprocessing_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_person_detection',
            executable='lidar_preprocessor',
            name='lidar_preprocessor',
            output='screen',
            parameters=[{
                'crop_min_x': -15.0,
                'crop_max_x': 15.0,
                'crop_min_y': -15.0,
                'crop_max_y': 15.0,
                'crop_min_z': -2.0,
                'crop_max_z': 4.0,
                'voxel_size': 0.15,
                'sor_nb_neighbors': 30,
                'sor_std_ratio': 1.5,
                'ground_z_threshold': -1.0,

                'dbscan_eps': 0.6,
                'dbscan_min_points': 20,
                'track_max_distance': 1.2,
                'track_max_age': 10,
                'track_min_hits': 3,
                'ema_alpha': 0.4,
            }],
            remappings=[],
        )
    ])