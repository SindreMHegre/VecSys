FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL [ "bin/bash", "-c" ]

RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add the ROS key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full

# Install ROS Noetic dependencies
RUN apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Initialize rosdep
RUN rosdep init && rosdep update

# Source ROS setup script
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Copy the bag with the images
COPY files/elektroHall_flight1_minimum.bag /home/elektroHall_flight1_minimum.bag

# Make the catkin workspace
RUN cd /home && mkdir catkin_ws && cd catkin_ws && mkdir src \
    && source /opt/ros/noetic/setup.bash && catkin_make \
    && echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Copy the pointcloud_processor package and build
COPY pointcloud_processor /home/catkin_ws/src/pointcloud_processor

# Create the pointcloud_processor package
RUN source /opt/ros/noetic/setup.bash && source /home/catkin_ws/devel/setup.bash \
    && cd /home/catkin_ws \
    && catkin_make

# Set the entrypoint to bash
ENTRYPOINT [ "bin/bash", "-c", "source /opt/ros/noetic/setup.bash && roscore" ]