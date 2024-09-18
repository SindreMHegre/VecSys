#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/point_cloud2_iterator.h>

ros::Publisher pub;

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
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
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(cloud_filtered, output);

    // Keep the original message's timestamp and frame_id
    output.header.stamp = cloud_msg->header.stamp;
    output.header.frame_id = cloud_msg->header.frame_id;

    // Publish the filtered point cloud
    pub.publish(output);

    ROS_INFO("Published filtered pointcloud");
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "pointcloud_processor");
    ros::NodeHandle nh;

    ROS_INFO("Pointcloud Processor started");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/ouster/points", 1, pointcloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/processed_pointcloud", 1);

    // Spin
    ros::spin();

    return 0;
}
