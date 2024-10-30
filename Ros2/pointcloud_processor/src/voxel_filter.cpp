#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

class VoxelFilter : public rclcpp::Node {
public:
    VoxelFilter() : Node("voxel_filter") {
        // Create a ROS 2 subscriber for the input point cloud
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", 10, std::bind(&VoxelFilter::voxel_callback, this, std::placeholders::_1));

        // Create a ROS 2 publisher for the output point cloud
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/processed_pointcloud", 10);

        RCLCPP_INFO(this->get_logger(), "Voxel filter node started");
    }

private:
    void voxel_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // Create a container for the data.
        pcl::PCLPointCloud2* pcl_cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr pcl_cloud_ptr(pcl_cloud);

        // Convert the sensor_msgs/PointCloud2 data to pcl/PCLPointCloud2.
        pcl_conversions::toPCL(*cloud_msg, *pcl_cloud);

        // Perform voxel grid downsampling
        pcl::PCLPointCloud2 cloud_filtered;
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(pcl_cloud_ptr);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);  // Set the voxel (leaf) size
        sor.filter(cloud_filtered);

        // Convert filtered PCL data back to ROS message
        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_filtered, output);

        // Keep the original message's timestamp and frame_id
        output.header.stamp = cloud_msg->header.stamp;
        output.header.frame_id = cloud_msg->header.frame_id;

        // Publish the filtered point cloud
        pub_->publish(output);

        RCLCPP_INFO(this->get_logger(), "Published filtered pointcloud");
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<VoxelFilter>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}