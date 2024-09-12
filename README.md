# VecSys
Code to do the in class assignments in cyberphysical connected vehicle systems at NTNU

## Images
You can find the already built images here: https://hub.docker.com/repository/docker/sindrehegre/ros-noetic-vecsys/general

## Day 1
This is the image for the fist day:

docker pull sindrehegre/ros-noetic-vecsys:mission1-bag

docker run -it sindrehegre/ros-noetic-vecsys:mission1-bag

Then in two new terminals in the same container:

[1] rosrun image_processor image_processor.py

[2] rosbag play home/mission_1.bag

The image_processor will publish to /processed_image
