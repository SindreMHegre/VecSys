cmake_minimum_required(VERSION 3.0.2)
project(icp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions tf2 tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(icp src/icp.cpp)

target_link_libraries(icp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(icp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
