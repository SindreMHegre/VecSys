cmake_minimum_required(VERSION 3.0.2)
project(statistical_outlier_removal_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  std_srvs
)

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
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(statistical_outlier_removal_filter src/statistical_outlier_removal_filter.cpp)
target_link_libraries(statistical_outlier_removal_filter
  ${catkin_LIBRARIES}
)
add_dependencies(statistical_outlier_removal_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})