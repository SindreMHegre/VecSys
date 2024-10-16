## Assignment 1

### Implements a node which runs the ICP algorithm

docker pull sindrehegre/ros-noetic-vecsys:assignment1

docker run -it sindrehegre/ros-noetic-vecsys:assignment1

### In other docker terminals:

rosrun icp icp

rosbag play home/outster_lidar.bag

#### The icp node publishes to /icp_transform
