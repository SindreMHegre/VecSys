#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <statistical_outlier_removal_filter/StatisticalOutlierRemovalFilter.h>
#include <unistd.h>

sensor_msgs::PointCloud2 latest_cloud;
bool service_called = false;
bool cloud_received = false;

// Point cloud callback function
void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (!service_called){
        return;
    }else{
        latest_cloud = *cloud_msg;
        service_called = false;
        cloud_received = true;
        return;
    }
}

// Service callback function
bool removeStatisticalOutliers(statistical_outlier_removal_filter::StatisticalOutlierRemovalFilter::Request &req,
                               statistical_outlier_removal_filter::StatisticalOutlierRemovalFilter::Response &res) {

    service_called = true;
    usleep(10000);

    if (cloud_received) {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(latest_cloud, *pcl_cloud);

        // Perform statistical outlier removal
        pcl::PCLPointCloud2 cloud_filtered;
        pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(cloud_filtered);

        // Convert filtered PCL data back to ROS message
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_filtered, output);

        // Keep the original message's timestamp and frame_id
        output.header.stamp = latest_cloud.header.stamp;
        output.header.frame_id = latest_cloud.header.frame_id;

        // Set the response
        res.filtered_cloud = output;
        res.success = true;
    } else {
        res.success = false;
    }
    cloud_received = false;

    return true;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "statistical_outlier_removal_filter");
    ros::NodeHandle nh;
    ROS_INFO("Statistical Outlier Removal Filter Server started");

    // Subscribe to the point cloud topic
    ros::Subscriber sub = nh.subscribe("/ouster/points", 1, pointcloud_callback);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("remove_statistical_outliers", removeStatisticalOutliers);

    // Spin to keep the service available
    ros::spin();

    return 0;
}