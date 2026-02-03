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

class EuclideanDetectorNode : public rclcpp::Node
{
public:
    EuclideanDetectorNode() : Node("euclidean_detector_node")
    {
        // Parameters
        this->declare_parameter("cluster_tolerance", 0.5);
        this->declare_parameter("min_cluster_size", 10);
        this->declare_parameter("max_cluster_size", 10000);
        this->declare_parameter("downsample_leaf_size", 0.1);
        this->declare_parameter("ransac_distance_threshold", 0.15);
        this->declare_parameter("ransac_max_iterations", 50);
        this->declare_parameter("max_z_threshold", 2.5);
        this->declare_parameter("use_height_extrapolation", true);
        
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        leaf_size_ = this->get_parameter("downsample_leaf_size").as_double();
        ransac_thresh_ = this->get_parameter("ransac_distance_threshold").as_double();
        ransac_iter_ = this->get_parameter("ransac_max_iterations").as_int();
        max_z_ = this->get_parameter("max_z_threshold").as_double();
        use_height_extrapolation_ = this->get_parameter("use_height_extrapolation").as_bool();
        
        // Subscriber and Publisher
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&EuclideanDetectorNode::cloudCallback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detected_objects", 10);
        
        RCLCPP_INFO(this->get_logger(), "Euclidean Detector Node initialized");
        RCLCPP_INFO(this->get_logger(), "Cluster tolerance: %.2f, Min size: %d", 
                    cluster_tolerance_, min_cluster_size_);
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
        // RANSAC ground plane removal
        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
        
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
        ransac.setDistanceThreshold(ransac_thresh_);
        ransac.setMaxIterations(ransac_iter_);
        ransac.computeModel();
        
        std::vector<int> inliers;
        ransac.getInliers(inliers);
        
        // Extract non-ground points
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
        inlier_indices->indices = inliers;
        extract.setIndices(inlier_indices);
        extract.setNegative(true);
        extract.filter(*non_ground);
        
        // Apply ceiling filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : non_ground->points) {
            if (point.z < max_z_) {
                filtered->push_back(point);
            }
        }
        
        return filtered;
    }
    
    std::vector<pcl::PointIndices> clusterCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // Create KD-tree for efficient neighbor search
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        
        // Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        
        return cluster_indices;
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
        
        // Height extrapolation
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
        
        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Downsample (optional but recommended for speed)
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = downsampleCloud(cloud);
        
        // Remove ground plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = removeGroundPlane(downsampled);
        
        // Cluster
        std::vector<pcl::PointIndices> clusters = clusterCloud(filtered);
        
        // Compute bounding boxes
        std::vector<BoundingBox> bboxes;
        for (const auto& cluster : clusters) {
            bboxes.push_back(computeBoundingBox(filtered, cluster));
        }
        
        // Publish markers
        publishBoundingBoxes(bboxes, msg->header);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Processed %zu→%zu→%zu objects in %ld ms",
                    cloud->points.size(), filtered->points.size(), 
                    bboxes.size(), duration.count());
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
            
            text_marker.text = "L:" + std::to_string(bbox.size_x).substr(0, 4) + 
                              " W:" + std::to_string(bbox.size_y).substr(0, 4) +
                              " H:" + std::to_string(bbox.size_z).substr(0, 4);
            
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            marker_array.markers.push_back(text_marker);
        }
        
        pub_->publish(marker_array);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double leaf_size_;
    double ransac_thresh_;
    int ransac_iter_;
    double max_z_;
    bool use_height_extrapolation_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EuclideanDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
