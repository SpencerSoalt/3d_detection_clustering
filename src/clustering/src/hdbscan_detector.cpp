#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>

// HDBSCAN implementation
// Note: This is a simplified version optimized for real-time performance
// For production, consider using a Python wrapper with scikit-learn-extra

class HDBSCANDetectorNode : public rclcpp::Node
{
public:
    HDBSCANDetectorNode() : Node("hdbscan_detector_node")
    {
        // Parameters
        this->declare_parameter("input_topic", "/patchworkpp/nonground");  // Patchwork output topic
        this->declare_parameter("min_cluster_size", 10);
        this->declare_parameter("min_samples", 5);
        this->declare_parameter("cluster_selection_epsilon", 0.0);
        this->declare_parameter("downsample_leaf_size", 0.1);
        this->declare_parameter("max_z_threshold", 2.5);
        this->declare_parameter("use_height_extrapolation", true);
        
        // ROI filtering
        this->declare_parameter("roi_min_x", -100.0);
        this->declare_parameter("roi_max_x", 100.0);
        this->declare_parameter("roi_min_y", -20.0);
        this->declare_parameter("roi_max_y", 20.0);
        this->declare_parameter("use_roi_filter", true);
        
        // Shape filtering
        this->declare_parameter("min_bbox_height", 0.3);
        this->declare_parameter("max_bbox_width", 5.0);
        this->declare_parameter("max_bbox_length", 10.0);
        this->declare_parameter("use_shape_filter", true);
        this->declare_parameter("min_point_density", 0.0);  // Points per cubic meter (0 = disabled)
        
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        min_samples_ = this->get_parameter("min_samples").as_int();
        cluster_selection_epsilon_ = this->get_parameter("cluster_selection_epsilon").as_double();
        leaf_size_ = this->get_parameter("downsample_leaf_size").as_double();
        max_z_ = this->get_parameter("max_z_threshold").as_double();
        use_height_extrapolation_ = this->get_parameter("use_height_extrapolation").as_bool();
        
        roi_min_x_ = this->get_parameter("roi_min_x").as_double();
        roi_max_x_ = this->get_parameter("roi_max_x").as_double();
        roi_min_y_ = this->get_parameter("roi_min_y").as_double();
        roi_max_y_ = this->get_parameter("roi_max_y").as_double();
        use_roi_filter_ = this->get_parameter("use_roi_filter").as_bool();
        
        min_bbox_height_ = this->get_parameter("min_bbox_height").as_double();
        max_bbox_width_ = this->get_parameter("max_bbox_width").as_double();
        max_bbox_length_ = this->get_parameter("max_bbox_length").as_double();
        use_shape_filter_ = this->get_parameter("use_shape_filter").as_bool();
        min_point_density_ = this->get_parameter("min_point_density").as_double();
        
        std::string input_topic = this->get_parameter("input_topic").as_string();
        
        // Subscriber and Publisher
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, 10,
            std::bind(&HDBSCANDetectorNode::cloudCallback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detected_objects", 10);
        
        RCLCPP_INFO(this->get_logger(), "HDBSCAN Detector Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s (Patchwork output)", input_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "min_cluster_size: %d, min_samples: %d", 
                    min_cluster_size_, min_samples_);
        if (use_roi_filter_) {
            RCLCPP_INFO(this->get_logger(), "ROI: X[%.1f, %.1f], Y[%.1f, %.1f]", 
                       roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_);
        }
        if (use_shape_filter_) {
            RCLCPP_INFO(this->get_logger(), "Shape filter: height>%.1fm, width<%.1fm, length<%.1fm",
                       min_bbox_height_, max_bbox_width_, max_bbox_length_);
            if (min_point_density_ > 0.0) {
                RCLCPP_INFO(this->get_logger(), "Density filter: >%.1f pts/m³", min_point_density_);
            }
        }
    }

private:
    struct BoundingBox
    {
        float min_x, max_x;
        float min_y, max_y;
        float min_z, max_z;
        float center_x, center_y, center_z;
        float size_x, size_y, size_z;
        int num_points;
    };
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        
        if (leaf_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(cloud);
            voxel_grid.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            voxel_grid.filter(*downsampled);
            return downsampled;
        }
        
        return cloud;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyFilters(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Apply ROI and ceiling filters only
        // NOTE: Ground removal is done by Patchwork upstream
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        for (const auto& point : cloud->points) {
            // Ceiling filter: Remove overhead objects (bridges, signs, tree branches)
            // This is NOT ground removal - it removes things ABOVE max_z
            if (point.z >= max_z_) continue;
            
            // ROI filter: Crop to region of interest (road area)
            if (use_roi_filter_) {
                if (point.x < roi_min_x_ || point.x > roi_max_x_) continue;
                if (point.y < roi_min_y_ || point.y > roi_max_y_) continue;
            }
            
            filtered->push_back(point);
        }
        
        return filtered;
    }
    
    // Compute mutual reachability distance
    std::vector<float> computeCoreDistances(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree)
    {
        std::vector<float> core_distances(cloud->points.size());
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            std::vector<int> neighbors;
            std::vector<float> distances;
            
            // Find k-nearest neighbors (k = min_samples)
            tree->nearestKSearch(i, min_samples_, neighbors, distances);
            
            // Core distance is distance to k-th nearest neighbor
            core_distances[i] = std::sqrt(distances[min_samples_ - 1]);
        }
        
        return core_distances;
    }
    
    // Simplified HDBSCAN using single-linkage clustering + density threshold
    std::vector<pcl::PointIndices> hdbscanClustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Build KD-tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        
        // Compute core distances (local density)
        auto core_distances = computeCoreDistances(cloud, tree);
        
        // Build minimum spanning tree based on mutual reachability distance
        // For real-time performance, we use a simplified approach:
        // Adaptive DBSCAN with local density-based eps
        
        std::vector<int> labels(cloud->points.size(), -1);
        int cluster_id = 0;
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (labels[i] != -1) continue;
            
            // Adaptive eps based on local density
            float local_eps = core_distances[i] * 1.5;  // Slightly larger than core distance
            if (cluster_selection_epsilon_ > 0.0) {
                local_eps = std::max(local_eps, static_cast<float>(cluster_selection_epsilon_));
            }
            
            // Find neighbors
            std::vector<int> neighbors;
            std::vector<float> distances;
            tree->radiusSearch(i, local_eps, neighbors, distances);
            
            // Check if core point (density-based)
            if (static_cast<int>(neighbors.size()) < min_samples_) {
                labels[i] = -2;  // Noise
                continue;
            }
            
            // Start new cluster
            labels[i] = cluster_id;
            
            // Expand cluster
            std::queue<int> to_process;
            for (int neighbor : neighbors) {
                if (labels[neighbor] == -1) {
                    to_process.push(neighbor);
                }
            }
            
            while (!to_process.empty()) {
                int current = to_process.front();
                to_process.pop();
                
                if (labels[current] != -1) continue;
                
                labels[current] = cluster_id;
                
                // Adaptive eps for current point
                float current_eps = core_distances[current] * 1.5;
                if (cluster_selection_epsilon_ > 0.0) {
                    current_eps = std::max(current_eps, static_cast<float>(cluster_selection_epsilon_));
                }
                
                std::vector<int> current_neighbors;
                std::vector<float> current_distances;
                tree->radiusSearch(current, current_eps, current_neighbors, current_distances);
                
                if (static_cast<int>(current_neighbors.size()) >= min_samples_) {
                    for (int neighbor : current_neighbors) {
                        if (labels[neighbor] == -1) {
                            to_process.push(neighbor);
                        }
                    }
                }
            }
            
            cluster_id++;
        }
        
        // Convert to PointIndices
        std::map<int, pcl::PointIndices> cluster_map;
        for (size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] >= 0) {
                cluster_map[labels[i]].indices.push_back(i);
            }
        }
        
        // Filter by cluster size
        std::vector<pcl::PointIndices> clusters;
        for (auto& pair : cluster_map) {
            if (static_cast<int>(pair.second.indices.size()) >= min_cluster_size_) {
                clusters.push_back(pair.second);
            }
        }
        
        return clusters;
    }
    
    BoundingBox computeBoundingBox(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointIndices& indices)
    {
        BoundingBox bbox;
        bbox.min_x = bbox.min_y = bbox.min_z = std::numeric_limits<float>::max();
        bbox.max_x = bbox.max_y = bbox.max_z = std::numeric_limits<float>::lowest();
        bbox.num_points = indices.indices.size();
        
        for (int idx : indices.indices) {
            const auto& point = cloud->points[idx];
            bbox.min_x = std::min(bbox.min_x, point.x);
            bbox.max_x = std::max(bbox.max_x, point.x);
            bbox.min_y = std::min(bbox.min_y, point.y);
            bbox.max_y = std::max(bbox.max_y, point.y);
            bbox.min_z = std::min(bbox.min_z, point.z);
            bbox.max_z = std::max(bbox.max_z, point.z);
        }
        
        // Height extrapolation: Extend box down to estimated ground
        // WHY: 16-channel LiDAR only hits TOP of objects (sparse vertically)
        // RESULT: Boxes look unrealistically flat (0.5m tall instead of 1.5m)
        // FIX: Assume object sits on ground, extend min_z downward
        if (use_height_extrapolation_ && bbox.max_z > 0.3f) {
            float estimated_ground = std::max(bbox.min_z - 0.3f, -0.5f);
            bbox.min_z = estimated_ground;
        }
        
        bbox.center_x = (bbox.min_x + bbox.max_x) / 2.0;
        bbox.center_y = (bbox.min_y + bbox.max_y) / 2.0;
        bbox.center_z = (bbox.min_z + bbox.max_z) / 2.0;
        bbox.size_x = bbox.max_x - bbox.min_x;
        bbox.size_y = bbox.max_y - bbox.min_y;
        bbox.size_z = bbox.max_z - bbox.min_z;
        
        return bbox;
    }
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = downsampleCloud(cloud);
        
        // Apply ROI and height filters (ground already removed by Patchwork)
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = applyFilters(downsampled);
        
        if (filtered->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No points after filtering");
            return;
        }
        
        // HDBSCAN clustering
        std::vector<pcl::PointIndices> clusters = hdbscanClustering(filtered);
        
        // Compute bounding boxes
        std::vector<BoundingBox> bboxes;
        for (const auto& cluster : clusters) {
            bboxes.push_back(computeBoundingBox(filtered, cluster));
        }
        
        // Shape filtering
        if (use_shape_filter_) {
            std::vector<BoundingBox> filtered_bboxes;
            for (const auto& bbox : bboxes) {
                // Height filter (too flat)
                if (bbox.size_z < min_bbox_height_) continue;
                
                // Width filter (too wide)
                if (bbox.size_y > max_bbox_width_) continue;
                
                // Length filter (too long)
                if (bbox.size_x > max_bbox_length_) continue;
                
                // Density filter (sparse objects like trees/bushes)
                if (min_point_density_ > 0.0) {
                    float volume = bbox.size_x * bbox.size_y * bbox.size_z;
                    if (volume > 0.01) {  // Avoid divide by zero
                        float density = bbox.num_points / volume;
                        if (density < min_point_density_) {
                            continue;  // Too sparse (likely vegetation)
                        }
                    }
                }
                
                filtered_bboxes.push_back(bbox);
            }
            bboxes = filtered_bboxes;
        }
        
        publishBoundingBoxes(bboxes, msg->header);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Processed %zu→%zu→%zu objects in %ld ms (%.1f Hz)",
                    cloud->points.size(), filtered->points.size(), 
                    bboxes.size(), duration.count(), 1000.0 / duration.count());
    }
    
    void publishBoundingBoxes(const std::vector<BoundingBox>& bboxes,
                              const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < bboxes.size(); ++i) {
            const auto& bbox = bboxes[i];
            
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "bounding_boxes";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = bbox.center_x;
            marker.pose.position.y = bbox.center_y;
            marker.pose.position.z = bbox.center_z;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = bbox.size_x;
            marker.scale.y = bbox.size_y;
            marker.scale.z = bbox.size_z;
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            
            marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            marker_array.markers.push_back(marker);
            
            visualization_msgs::msg::Marker text_marker;
            text_marker.header = header;
            text_marker.ns = "labels";
            text_marker.id = i + 10000;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            
            text_marker.pose.position.x = bbox.center_x;
            text_marker.pose.position.y = bbox.center_y;
            text_marker.pose.position.z = bbox.max_z + 0.3;
            text_marker.pose.orientation.w = 1.0;
            
            text_marker.scale.z = 0.5;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            
            text_marker.text = "Pts:" + std::to_string(bbox.num_points);
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            marker_array.markers.push_back(text_marker);
        }
        
        pub_->publish(marker_array);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    
    int min_cluster_size_;
    int min_samples_;
    double cluster_selection_epsilon_;
    double leaf_size_;
    double max_z_;
    bool use_height_extrapolation_;
    
    double roi_min_x_;
    double roi_max_x_;
    double roi_min_y_;
    double roi_max_y_;
    bool use_roi_filter_;
    
    double min_bbox_height_;
    double max_bbox_width_;
    double max_bbox_length_;
    bool use_shape_filter_;
    double min_point_density_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HDBSCANDetectorNode>());
    rclcpp::shutdown();
    return 0;
}