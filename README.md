




### Configure Topic `src/clustering/src/dbscan_detector.cpp`
```
        // Subscriber and Publisher
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&DBSCANDetectorNode::cloudCallback, this, std::placeholders::_1));
```


### Configure Parameters `src/clustering/launch/dbscan_detector.launch.py`
* `eps`: DBSCAN neighbor radius (m)
* `min_points`: DBSCAN core-point minimum neighbors
* `downsample_leaf_size`: voxel-grid downsample size (m)
* `min_cluster_size`: reject clusters with fewer points
* `max_cluster_size`: reject clusters with too many points
* `ransac_distance_threshold`: ground-plane inlier distance (m)
* `ransac_max_iterations`: RANSAC iteration limit
* `max_z_threshold`: max height cutoff used for filtering (m)
* `use_height_extrapolation`: enable height extrapolation
* `use_roi_filter`: enable ROI cropping
* `roi_min_x`: ROI min X (m)
* `roi_max_x`: ROI max X (m)
* `roi_min_y`: ROI min Y (m)
* `roi_max_y`: ROI max Y (m)
* `use_shape_filter`: enable bbox shape filtering
* `min_bbox_height`: minimum bbox height allowed (m)
* `max_bbox_width`: maximum bbox width allowed (m)
* `max_bbox_length`: maximum bbox length allowed (m)



### Run Nodes
```
ros2 launch clustering dbscan_detector.launch.py

ros2 launch clustering euclidean_detector.launch.py

ros2 launch clustering voxel_detector.launch.py
```
