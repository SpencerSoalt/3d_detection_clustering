#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>
#include <unordered_set>

class DBSCANDetectorNode : public rclcpp::Node
{
public:
    DBSCANDetectorNode() : Node("dbscan_detector_node")
    {
        // Parameters
        this->declare_parameter("eps", 0.5);                          // Neighborhood radius
        this->declare_parameter("min_points", 5);                     // Min points for core point
        this->declare_parameter("downsample_leaf_size", 0.1);
        this->declare_parameter("min_cluster_size", 10);
        this->declare_parameter("max_cluster_size", 10000);
        this->declare_parameter("ransac_distance_threshold", 0.15);
        this->declare_parameter("ransac_max_iterations", 50);
        this->declare_parameter("max_z_threshold", 2.5);
        this->declare_parameter("use_height_extrapolation", true);
        
        // ROI filtering (Region of Interest)
        this->declare_parameter("roi_min_x", -100.0);                 // Min X (behind vehicle)
        this->declare_parameter("roi_max_x", 100.0);                  // Max X (front of vehicle)
        this->declare_parameter("roi_min_y", -20.0);                  // Min Y (left side)
        this->declare_parameter("roi_max_y", 20.0);                   // Max Y (right side)
        this->declare_parameter("use_roi_filter", true);              // Enable/disable ROI
        
        // Shape filtering (filter flat/wide boxes)
        this->declare_parameter("min_bbox_height", 0.3);              // Min height (meters)
        this->declare_parameter("max_bbox_width", 5.0);               // Max width (meters)
        this->declare_parameter("max_bbox_length", 10.0);             // Max length (meters)
        this->declare_parameter("use_shape_filter", true);            // Enable/disable shape filter
        
        eps_ = this->get_parameter("eps").as_double();
        min_points_ = this->get_parameter("min_points").as_int();
        leaf_size_ = this->get_parameter("downsample_leaf_size").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        ransac_thresh_ = this->get_parameter("ransac_distance_threshold").as_double();
        ransac_iter_ = this->get_parameter("ransac_max_iterations").as_int();
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
        
        // Subscriber and Publisher
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&DBSCANDetectorNode::cloudCallback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detected_objects", 10);
        
        RCLCPP_INFO(this->get_logger(), "DBSCAN Detector Node initialized");
        RCLCPP_INFO(this->get_logger(), "eps: %.2f, min_points: %d", eps_, min_points_);
        if (use_roi_filter_) {
            RCLCPP_INFO(this->get_logger(), "ROI: X[%.1f, %.1f], Y[%.1f, %.1f]", 
                       roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_);
        }
        if (use_shape_filter_) {
            RCLCPP_INFO(this->get_logger(), "Shape filter: height>%.1fm, width<%.1fm, length<%.1fm",
                       min_bbox_height_, max_bbox_width_, max_bbox_length_);
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
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
        
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
        ransac.setDistanceThreshold(ransac_thresh_);
        ransac.setMaxIterations(ransac_iter_);
        ransac.computeModel();
        
        std::vector<int> inliers;
        ransac.getInliers(inliers);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
        inlier_indices->indices = inliers;
        extract.setIndices(inlier_indices);
        extract.setNegative(true);
        extract.filter(*non_ground);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : non_ground->points) {
            // Apply height filter
            if (point.z >= max_z_) continue;
            
            // Apply ROI filter
            if (use_roi_filter_) {
                if (point.x < roi_min_x_ || point.x > roi_max_x_) continue;
                if (point.y < roi_min_y_ || point.y > roi_max_y_) continue;
            }
            
            filtered->push_back(point);
        }
        
        return filtered;
    }
    
    std::vector<pcl::PointIndices> dbscanClustering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Build KD-tree for efficient neighbor search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        
        // DBSCAN implementation
        std::vector<int> labels(cloud->points.size(), -1);  // -1 = unvisited, -2 = noise
        int cluster_id = 0;
        
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (labels[i] != -1) continue;  // Already processed
            
            // Find neighbors
            std::vector<int> neighbors;
            std::vector<float> distances;
            tree->radiusSearch(i, eps_, neighbors, distances);
            
            // Check if core point
            if (static_cast<int>(neighbors.size()) < min_points_) {
                labels[i] = -2;  // Mark as noise
                continue;
            }
            
            // Start new cluster
            labels[i] = cluster_id;
            
            // Expand cluster (BFS)
            std::vector<int> seeds = neighbors;
            size_t seed_idx = 0;
            
            while (seed_idx < seeds.size()) {
                int current = seeds[seed_idx++];
                
                if (labels[current] == -2) {
                    labels[current] = cluster_id;  // Change noise to border point
                }
                
                if (labels[current] != -1) continue;  // Already processed
                
                labels[current] = cluster_id;
                
                // Find neighbors of current point
                std::vector<int> current_neighbors;
                std::vector<float> current_distances;
                tree->radiusSearch(current, eps_, current_neighbors, current_distances);
                
                // If core point, add its neighbors to seeds
                if (static_cast<int>(current_neighbors.size()) >= min_points_) {
                    for (int neighbor : current_neighbors) {
                        if (labels[neighbor] == -1) {
                            seeds.push_back(neighbor);
                        }
                    }
                }
            }
            
            cluster_id++;
        }
        
        // Convert labels to PointIndices format
        std::vector<pcl::PointIndices> clusters;
        std::map<int, pcl::PointIndices> cluster_map;
        
        for (size_t i = 0; i < labels.size(); ++i) {
            if (labels[i] >= 0) {  // Not noise
                cluster_map[labels[i]].indices.push_back(i);
            }
        }
        
        // Filter by size and convert to vector
        for (auto& pair : cluster_map) {
            int size = pair.second.indices.size();
            if (size >= min_cluster_size_ && size <= max_cluster_size_) {
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
        
        // Convert
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = downsampleCloud(cloud);
        
        // Remove ground
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = removeGroundPlane(downsampled);
        
        if (filtered->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No points after ground removal");
            return;
        }
        
        // DBSCAN clustering
        std::vector<pcl::PointIndices> clusters = dbscanClustering(filtered);
        
        // Compute bounding boxes
        std::vector<BoundingBox> bboxes;
        for (const auto& cluster : clusters) {
            bboxes.push_back(computeBoundingBox(filtered, cluster));
        }
        
        // Apply shape filtering
        if (use_shape_filter_) {
            std::vector<BoundingBox> filtered_bboxes;
            for (const auto& bbox : bboxes) {
                // Filter out flat boxes (ground plane artifacts, road markings)
                if (bbox.size_z < min_bbox_height_) {
                    continue;  // Too flat
                }
                
                // Filter out very wide boxes (likely noise or merged objects)
                if (bbox.size_y > max_bbox_width_) {
                    continue;  // Too wide
                }
                
                // Filter out very long boxes (walls, barriers)
                if (bbox.size_x > max_bbox_length_) {
                    continue;  // Too long
                }
                
                filtered_bboxes.push_back(bbox);
            }
            bboxes = filtered_bboxes;
        }
        
        // Publish
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
            
            // Cube marker
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
            
            // Text marker
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
            
            text_marker.text = "Pts:" + std::to_string(bbox.num_points) + "\n" +
                              "L:" + std::to_string(bbox.size_x).substr(0, 4) + 
                              " W:" + std::to_string(bbox.size_y).substr(0, 4);
            
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            marker_array.markers.push_back(text_marker);
        }
        
        pub_->publish(marker_array);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    
    double eps_;
    int min_points_;
    double leaf_size_;
    int min_cluster_size_;
    int max_cluster_size_;
    double ransac_thresh_;
    int ransac_iter_;
    double max_z_;
    bool use_height_extrapolation_;
    
    // ROI filtering
    double roi_min_x_;
    double roi_max_x_;
    double roi_min_y_;
    double roi_max_y_;
    bool use_roi_filter_;
    
    // Shape filtering
    double min_bbox_height_;
    double max_bbox_width_;
    double max_bbox_length_;
    bool use_shape_filter_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DBSCANDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
