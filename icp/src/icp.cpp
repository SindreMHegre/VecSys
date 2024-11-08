#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

class ICPNode : public rclcpp::Node {
public:
    ICPNode() : Node("icp_node") {
        // Create a ROS 2 subscriber for the input point cloud
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/os1_cloud_node/points", 1, std::bind(&ICPNode::icp_callback, this, std::placeholders::_1));

        // Create a ROS 2 publisher for the transformation
        pub_transform_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/icp_transform", 1);

        RCLCPP_INFO(this->get_logger(), "ICP algorithm node started");
    }

private:
    // This callback is called every time a new point cloud is received.
    // It performs ICP between the previous and current pointclound and publishes the transformation between them.
    void icp_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // Convert the incoming point cloud from ROS format to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud_msg, *current_cloud);

        // Apply a voxel grid filter to downsample the point cloud to make the icp faster
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);
        voxel_grid.setInputCloud(current_cloud);
        voxel_grid.filter(*current_cloud);

        // Check if this is the first point cloud received
        if (!first_cloud_received_) {
            // Store the first point cloud for future ICP processing
            *previous_cloud_ = *current_cloud;
            first_cloud_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received the first point cloud, storing for future ICP.");
            return;
        }

        // Set up the ICP algorithm
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(previous_cloud_);
        icp.setInputTarget(current_cloud);

        // Comment in the following lines to adjust the ICP parameters
        // icp.setMaxCorrespondenceDistance(1.0);
        // icp.setTransformationEpsilon(1e-8);
        icp.setMaximumIterations(50);

        // Output point cloud to hold the ICP result
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud);

        if (icp.hasConverged()) {
            RCLCPP_INFO(this->get_logger(), "ICP converged with score: %f", icp.getFitnessScore());

            // Get the transformation matrix from the ICP result
            Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

            // Convert the transformation matrix to a TransformStamped message
            geometry_msgs::msg::TransformStamped transform_msg;
            // Keep the timestamp of current point cloud to match the transformation to the correct time
            transform_msg.header.stamp = cloud_msg->header.stamp;
            transform_msg.header.frame_id = "previous_frame"; // Frame ID of the previous point cloud
            transform_msg.child_frame_id = "current_frame";   // Frame ID of the current point cloud

            // Extract the translation
            transform_msg.transform.translation.x = transformation_matrix(0, 3);
            transform_msg.transform.translation.y = transformation_matrix(1, 3);
            transform_msg.transform.translation.z = transformation_matrix(2, 3);

            // Extract the rotation (as a quaternion)
            Eigen::Matrix3f rotation_matrix = transformation_matrix.block<3, 3>(0, 0);
            Eigen::Quaternionf quaternion(rotation_matrix);
            transform_msg.transform.rotation.x = quaternion.x();
            transform_msg.transform.rotation.y = quaternion.y();
            transform_msg.transform.rotation.z = quaternion.z();
            transform_msg.transform.rotation.w = quaternion.w();

            // Publish the transformation
            pub_transform_->publish(transform_msg);
            RCLCPP_INFO(this->get_logger(), "Published transformation from previous cloud to current cloud.");

            // Update the previous cloud to be the current cloud for the next iteration
            *previous_cloud_ = *current_cloud;
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_transform_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    bool first_cloud_received_ = false;
};

int main(int argc, char** argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<ICPNode>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
