#include <ros/ros.h>
#include <statistical_outlier_removal_filter/StatisticalOutlierRemovalFilter.h>

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "statistical_outlier_removal_client");
    ros::NodeHandle nh;

    // Create a service client
    ros::ServiceClient client = nh.serviceClient<statistical_outlier_removal_filter::StatisticalOutlierRemovalFilter>("remove_statistical_outliers");

    // Create a service request and response
    statistical_outlier_removal_filter::StatisticalOutlierRemovalFilter srv;

    // Call the service
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Service call succeeded: filtered point cloud received");

            sensor_msgs::PointCloud2 filtered_cloud = srv.response.filtered_cloud;
            // Use the filtered point cloud to whatever you need
        } else {
            ROS_WARN("Service call failed: server has not yet received a point cloud");
        }
    } else {
        ROS_ERROR("Failed to call service remove_statistical_outliers");
    }

    return 0;
}