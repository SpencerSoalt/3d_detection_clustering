from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clustering',
            executable='dbscan_detector',
            name='dbscan_detector',
            output='screen',
            parameters=[{
                # DBSCAN parameters
                'eps': 0.5,                             # Neighborhood radius (meters)
                                                         # Smaller = tighter clusters, more splits
                                                         # Larger = looser clusters, may merge
                                                         # Typical: 0.3-0.7
                
                'min_points': 5,                        # Min points for core point
                                                         # Lower = more sensitive, more noise
                                                         # Higher = filters noise, may miss objects
                                                         # Typical: 3-10
                
                # Downsampling (CRITICAL for speed)
                'downsample_leaf_size': 0.1,           # 10cm voxel grid
                                                        # Increase for more speed
                                                        # Decrease for more accuracy
                
                # Cluster filtering
                'min_cluster_size': 30,                 # Minimum points per cluster
                'max_cluster_size': 300,              # Maximum points per cluster
                
                # # Ground removal
                # 'ransac_distance_threshold': 0.15,
                # 'ransac_max_iterations': 50,

                'max_z_threshold': 1.98,             # Filter points above this height (meters)
                
                # Height extrapolation
                'use_height_extrapolation': True,
                
                # ROI filtering (Crop to road area)
                'use_roi_filter': True,                # Enable ROI filtering
                'roi_min_x': 0.0,                      # Min X (meters) - behind vehicle
                'roi_max_x': 50.0,                     # Max X (meters) - front of vehicle
                'roi_min_y': -7.0,                    # Min Y (meters) - left side (negative)
                'roi_max_y': 7.0,                     # Max Y (meters) - right side (positive)
            
             # Shape filtering (filter out flat/wide boxes)
                'use_shape_filter': True,             # Enable shape filtering
                'min_bbox_height': 0.5,               # Min height (meters) - ignore flat objects
                                                       # 0.2 = very strict (only tall objects)
                                                       # 0.3 = balanced (default)
                                                       # 0.5 = loose (allow some flat objects)
                
                'max_bbox_width': 5.0,                # Max width Y-axis (meters) - ignore wide objects
                                                       # 3.0 = strict (single lane)
                                                       # 5.0 = balanced (default, car width)
                                                       # 8.0 = loose (allow wide objects)
                
                'max_bbox_length': 10.0,              # Max length X-axis (meters) - ignore long objects
                                                       # 8.0 = strict (single car)
                                                       # 10.0 = balanced (default, truck/bus)
                                                       # 15.0 = loose (allow very long objects)
                'min_point_density': 30.0,            # Min points per m³ (filters vegetation!)
                                                        # Trees/bushes: 5-10 pts/m³ (sparse)
                                                        # Cars: 20-50 pts/m³ (dense)
                                                        # 15.0 = good balance
                                                        # Increase to 20-25 if still noisy
                                                        # Set to 0.0 to disable
                }]
        )
    ])


# Old Launch file (no ROI & shape filtering)

# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='clustering',
#             executable='dbscan_detector_node',
#             name='dbscan_detector',
#             output='screen',
#             parameters=[{
#                 # DBSCAN parameters
#                 'eps': 0.5,                             # Neighborhood radius (meters)
#                                                          # Smaller = tighter clusters, more splits
#                                                          # Larger = looser clusters, may merge
#                                                          # Typical: 0.3-0.7
                
#                 'min_points': 7,                        # Min points for core point
#                                                          # Lower = more sensitive, more noise
#                                                          # Higher = filters noise, may miss objects
#                                                          # Typical: 3-10
                
#                 # Downsampling (CRITICAL for speed)
#                 'downsample_leaf_size': 0.1,           # 10cm voxel grid
#                                                         # Increase to 0.15 for more speed
#                                                         # Decrease to 0.08 for more accuracy
                
#                 # Cluster filtering
#                 'min_cluster_size': 15,                 # Minimum points per cluster
#                 'max_cluster_size': 500,              # Maximum points per cluster
                
#                 # Ground removal
#                 'ransac_distance_threshold': 0.15,
#                 'ransac_max_iterations': 50,
#                 'max_z_threshold': 2.3,
                
#                 # Height extrapolation
#                 'use_height_extrapolation': True,
#             }]
#         )
#     ])
