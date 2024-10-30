# VecSys
Code to do the in class assignments in cyberphysical connected vehicle systems at NTNU

# Ros2
Since I'm able to install ros2 locally there will be no more docker images

## Day 2
The new voxelfilter can be found in Ros2/pointcloud_processor

## Images
You can find the already built images here: https://hub.docker.com/repository/docker/sindrehegre/ros-noetic-vecsys/general

## Day 4 Actions in ROS

This creates a action service that simulates moving a robot arm just to get to know actions in ROS.

docker pull sindrehegre/ros-noetic-vecsys:day4

## Day 3 Services on pointclouds

You know the drill

docker pull sindrehegre/ros-noetic-vecsys:day3

## Day 2 Topics with pointclouds

docker pull sindrehegre/ros-noetic-vecsys:day2

then mostly same as day1, image_processor changed name to pointcloud_processor and the bag has a new name

## Day 1 Topics with images
This is the image for the fist day:

docker pull sindrehegre/ros-noetic-vecsys:mission1-bag

docker run -it sindrehegre/ros-noetic-vecsys:mission1-bag

Then in two new terminals in the same container:

[1] rosrun image_processor image_processor.py

[2] rosbag play home/mission_1.bag

The image_processor will publish to /processed_image

