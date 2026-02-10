from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clustering',  # Or rename package to 'lidar_detector'
            executable='euclidean_detector',
            name='euclidean_detector',
            output='screen',
            parameters=[{
                # Clustering parameters
                'cluster_tolerance': 0.7,           # Max distance to group points (meters)
                'min_cluster_size': 15,             # Minimum points per cluster
                'max_cluster_size': 350,          # Maximum points per cluster
                
                # Downsampling (CRITICAL for speed)
                'downsample_leaf_size': 0.0,        # 10cm voxel grid for downsampling
                                                     # Set to 0.0 to disable (slower)
                                                     # Increase to 0.15 for more speed
                
                # Ground removal (RANSAC)
                'ransac_distance_threshold': 0.15,  # Points within 15cm of ground plane
                'ransac_max_iterations': 50,        # RANSAC iterations
                
                # Height filtering
                'max_z_threshold': 5.0,             # Filter points above this height
                
                # Height extrapolation (makes boxes less flat)
                'use_height_extrapolation': True,
            }]
        )
    ])
