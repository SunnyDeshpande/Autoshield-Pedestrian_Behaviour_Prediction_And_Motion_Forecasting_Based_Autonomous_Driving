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
                # Cropping parameters
                'crop_min_x': -15.0,
                'crop_max_x': 0.0,
                'crop_min_y': -3.0,
                'crop_max_y': 3.0,
                'crop_min_z': -2.1,
                'crop_max_z': 1.0,
                
                # Preprocessing parameters
                'voxel_size': 0.15,
                'sor_nb_neighbors': 30,
                'sor_std_ratio': 1.5,
                'ground_z_threshold': -1.0,

                # Clustering & Tracking parameters
                'dbscan_eps': 0.6,
                'dbscan_min_points': 20,
                'track_max_distance': 1.2,
                'track_max_age': 10,
                'track_min_hits': 3,
                'ema_alpha': 0.4,
                
                # Human detection parameters
                'human_height_min': 0.8,
                'human_height_max': 2.2,
                'human_width_max': 1.0,
                'human_depth_max': 1.0,
                'human_ratio_min': 1.0,
                'human_footprint_max': 0.5,
                'human_volume_min': 0.08,
                'human_volume_max': 1.2,
                'human_compactness_max': 0.4,
                'human_xy_flatness_min': 0.2,
                'min_motion_threshold': 0.3,
                'static_check_frames': 10,
                'max_intensity_avg': 99999.0,
            }],
            remappings=[],
        )
    ])