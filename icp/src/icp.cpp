#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

ros::Publisher pub_transform;

// Should take the last point cloud and calculate the transformation to the current point cloud and then publish for the tf library to use

// Store the previous point cloud to calculate the transformation
pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud(new pcl::PointCloud<pcl::PointXYZ>());
bool first_cloud_received = false;

void icp_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Convert the incoming point cloud from ROS format to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *current_cloud);

    // Apply a voxel grid filter to downsample the point cloud to make the icp faster
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(0.5f, 0.5f, 0.5f);
    voxel_grid.setInputCloud(current_cloud);
    voxel_grid.filter(*current_cloud);

    // Check if this is the first point cloud received
    if (!first_cloud_received) {
        // Store the first point cloud for future ICP processing
        *previous_cloud = *current_cloud;
        first_cloud_received = true;
        ROS_INFO("Received the first point cloud, storing for future ICP.");
        return;
    }

    // Perform ICP to find the transformation from the previous cloud to the current cloud
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(previous_cloud);
    icp.setInputTarget(current_cloud);

    // Comment in the following lines to adjust the ICP parameters
    // icp.setMaxCorrespondenceDistance(1.0);
    // icp.setTransformationEpsilon(1e-8);
    icp.setMaximumIterations(50);

    // Output point cloud to hold the ICP result
    pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
    icp.align(aligned_cloud);

    if (icp.hasConverged()) {
        ROS_INFO("ICP converged with score: %f", icp.getFitnessScore());

        // Get the transformation matrix from the ICP result
        Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

        // Convert the transformation matrix to a TransformStamped message
        geometry_msgs::TransformStamped transform_msg;
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
        pub_transform.publish(transform_msg);
        ROS_INFO("Published transformation from previous cloud to current cloud.");

        // Update the previous cloud to be the current cloud for the next iteration
        *previous_cloud = *current_cloud;
    } else {
        ROS_WARN("ICP did not converge.");
    }
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "icp");
    ros::NodeHandle nh;

    ROS_INFO("ICP algorithm node started");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/os_cloud_node/points", 1, icp_callback);

    // Create a ROS publisher for the transformation
    pub_transform = nh.advertise<geometry_msgs::TransformStamped>("/icp_transform", 1);

    // Spin
    ros::spin();

    return 0;
}
