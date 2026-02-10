from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clustering',
            executable='voxel_detector',
            name='voxel_detector',
            output='screen',
            parameters=[{
                'voxel_size': 0.10,              # 20cm voxels
                'min_points_per_cluster': 20,   # Minimum points to form a cluster
                'max_points_per_cluster': 100, # Maximum points in a cluster
                'ground_z_threshold': -5.0,     # Filter points below this (ground)
                'max_z_threshold': 2.0,         # Filter points above this (ceiling/noise)
            }]
        )
    ])
