#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <unordered_map>
#include <vector>
#include <queue>

class VoxelDetectorNode : public rclcpp::Node
{
public:
    VoxelDetectorNode() : Node("voxel_detector_node")
    {
        // Parameters
        this->declare_parameter("voxel_size", 0.2);
        this->declare_parameter("min_points_per_cluster", 10);
        this->declare_parameter("max_points_per_cluster", 10000);
        this->declare_parameter("ground_z_threshold", -1.5);
        this->declare_parameter("max_z_threshold", 2.0);
        
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        min_points_ = this->get_parameter("min_points_per_cluster").as_int();
        max_points_ = this->get_parameter("max_points_per_cluster").as_int();
        ground_z_ = this->get_parameter("ground_z_threshold").as_double();
        max_z_ = this->get_parameter("max_z_threshold").as_double();
        
        // Subscriber and Publisher
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&VoxelDetectorNode::cloudCallback, this, std::placeholders::_1));
        
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/detected_objects", 10);
        
        RCLCPP_INFO(this->get_logger(), "Voxel Detector Node initialized");
        RCLCPP_INFO(this->get_logger(), "Voxel size: %.2f, Min points: %d", voxel_size_, min_points_);
    }

private:
    struct VoxelKey
    {
        int x, y, z;
        
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };
    
    struct VoxelKeyHash
    {
        std::size_t operator()(const VoxelKey& k) const {
            // Simple hash combination
            return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
        }
    };
    
    struct BoundingBox
    {
        float min_x, max_x;
        float min_y, max_y;
        float min_z, max_z;
        float center_x, center_y, center_z;
        float size_x, size_y, size_z;
        int num_points;
    };
    
    VoxelKey pointToVoxel(const pcl::PointXYZ& point)
    {
        return {
            static_cast<int>(std::floor(point.x / voxel_size_)),
            static_cast<int>(std::floor(point.y / voxel_size_)),
            static_cast<int>(std::floor(point.z / voxel_size_))
        };
    }
    
    std::vector<VoxelKey> getNeighbors(const VoxelKey& voxel)
    {
        std::vector<VoxelKey> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    neighbors.push_back({voxel.x + dx, voxel.y + dy, voxel.z + dz});
                }
            }
        }
        return neighbors;
    }
    
    std::vector<std::vector<VoxelKey>> clusterVoxels(
        std::unordered_map<VoxelKey, std::vector<pcl::PointXYZ>, VoxelKeyHash>& voxel_map)
    {
        std::vector<std::vector<VoxelKey>> clusters;
        std::unordered_set<VoxelKey, VoxelKeyHash> visited;
        
        for (auto& [voxel, points] : voxel_map) {
            if (visited.count(voxel)) continue;
            
            // BFS clustering
            std::vector<VoxelKey> cluster;
            std::queue<VoxelKey> queue;
            queue.push(voxel);
            visited.insert(voxel);
            
            while (!queue.empty()) {
                VoxelKey current = queue.front();
                queue.pop();
                cluster.push_back(current);
                
                // Check neighbors
                for (const auto& neighbor : getNeighbors(current)) {
                    if (visited.count(neighbor)) continue;
                    if (voxel_map.find(neighbor) != voxel_map.end()) {
                        visited.insert(neighbor);
                        queue.push(neighbor);
                    }
                }
            }
            
            // Count total points in cluster
            int total_points = 0;
            for (const auto& v : cluster) {
                total_points += voxel_map[v].size();
            }
            
            if (total_points >= min_points_ && total_points <= max_points_) {
                clusters.push_back(cluster);
            }
        }
        
        return clusters;
    }
    
    BoundingBox computeBoundingBox(
        const std::vector<VoxelKey>& cluster,
        std::unordered_map<VoxelKey, std::vector<pcl::PointXYZ>, VoxelKeyHash>& voxel_map)
    {
        BoundingBox bbox;
        bbox.min_x = bbox.min_y = bbox.min_z = std::numeric_limits<float>::max();
        bbox.max_x = bbox.max_y = bbox.max_z = std::numeric_limits<float>::lowest();
        bbox.num_points = 0;
        
        for (const auto& voxel : cluster) {
            for (const auto& point : voxel_map[voxel]) {
                bbox.min_x = std::min(bbox.min_x, point.x);
                bbox.max_x = std::max(bbox.max_x, point.x);
                bbox.min_y = std::min(bbox.min_y, point.y);
                bbox.max_y = std::max(bbox.max_y, point.y);
                bbox.min_z = std::min(bbox.min_z, point.z);
                bbox.max_z = std::max(bbox.max_z, point.z);
                bbox.num_points++;
            }
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
        
        // Voxelize and filter ground points
        std::unordered_map<VoxelKey, std::vector<pcl::PointXYZ>, VoxelKeyHash> voxel_map;
        
        for (const auto& point : cloud->points) {
            // Filter ground and ceiling
            if (point.z < ground_z_ || point.z > max_z_) continue;
            
            VoxelKey voxel = pointToVoxel(point);
            voxel_map[voxel].push_back(point);
        }
        
        // Cluster voxels
        auto clusters = clusterVoxels(voxel_map);
        
        // Compute bounding boxes
        std::vector<BoundingBox> bboxes;
        for (const auto& cluster : clusters) {
            bboxes.push_back(computeBoundingBox(cluster, voxel_map));
        }
        
        // Publish markers
        publishBoundingBoxes(bboxes, msg->header);
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        RCLCPP_DEBUG(this->get_logger(), "Processed %zu points -> %zu objects in %ld ms",
                    cloud->points.size(), bboxes.size(), duration.count());
    }
    
    void publishBoundingBoxes(const std::vector<BoundingBox>& bboxes,
                              const std_msgs::msg::Header& header)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < bboxes.size(); ++i) {
            const auto& bbox = bboxes[i];
            
            // Create cube marker
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
            
            // Create text marker with info
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
    
    double voxel_size_;
    int min_points_;
    int max_points_;
    double ground_z_;
    double max_z_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
