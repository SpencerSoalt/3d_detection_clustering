from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clustering',
            executable='hdbscan_detector',
            name='hdbscan_detector',
            output='screen',
            parameters=[{
                # HDBSCAN parameters (KEY TUNING)
                'min_cluster_size': 10,                 # Min points to form valid cluster
                                                         # Lower = more sensitive (detects small objects)
                                                         # Higher = more strict (filters noise)
                                                         # Typical: 8-15
                
                'min_samples': 5,                       # Min neighbors for core point
                                                         # Lower = more clusters
                                                         # Higher = denser clusters only
                                                         # Typical: 3-8
                
                'cluster_selection_epsilon': 0.0,       # Minimum distance threshold
                                                         # 0.0 = fully adaptive (default)
                                                         # 0.3-0.5 = enforce minimum separation
                
        # Downsampling (CRITICAL for speed)
        'downsample_leaf_size': 0.1,           # 10cm voxel grid
                                                # Increase to 0.15 for more speed
                                                # Decrease to 0.08 for more accuracy
                
        # Ceiling filter (NOT ground removal - that's done by Patchwork)
        'max_z_threshold': 2.75,                # Remove overhead objects (bridges, signs)
                                                # Keeps objects below 2.5m above ground
                                                # Adjust based on tallest vehicle you care about
                
        # Height extrapolation (fix flat boxes from sparse LiDAR)
        'use_height_extrapolation': False,      # Extend boxes down to estimated ground
                                                # Fixes: 16-channel only hits top of objects
                                                # Result: More realistic box heights
                                                # Set False if you trust Patchwork ground fully
                
                # ROI filtering
                'use_roi_filter': False,
                'roi_min_x': 0.0,
                'roi_max_x': 50.0,
                'roi_min_y': -10.0,
                'roi_max_y': 10.0,
                
                # Shape filtering
                'use_shape_filter': True,
                'min_bbox_height': 0.3,
                'max_bbox_width': 5.0,
                'max_bbox_length': 10.0,
                'min_point_density': 15.0,             # Min points per m³ (filters vegetation!)
                                                        # Trees/bushes: 5-10 pts/m³ (sparse)
                                                        # Cars: 20-50 pts/m³ (dense)
                                                        # 15.0 = good balance
                                                        # Increase to 20-25 if still noisy
                                                        # Set to 0.0 to disable
            }]
        )
    ])