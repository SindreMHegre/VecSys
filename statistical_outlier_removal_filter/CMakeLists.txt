cmake_minimum_required(VERSION 3.0.2)
project(statistical_outlier_removal_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  message_generation
  std_srvs
)

find_package(PCL REQUIRED)

add_service_files(
  FILES
  StatisticalOutlierRemovalFilter.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions message_runtime std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Link PCL libraries
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(statistical_outlier_removal_filter src/statistical_outlier_removal_filter.cpp)
target_link_libraries(statistical_outlier_removal_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(statistical_outlier_removal_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(statistical_outlier_removal_filter PUBLIC cxx_std_17)

add_executable(service_client src/service_client.cpp)
target_link_libraries(service_client ${catkin_LIBRARIES})
add_dependencies(service_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(service_client PUBLIC cxx_std_17)