cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_processor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_processor src/pointcloud_processor.cpp)

target_link_libraries(pointcloud_processor
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(pointcloud_processor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
