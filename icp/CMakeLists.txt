cmake_minimum_required(VERSION 3.8)
project(icp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
  add_compile_options(-Wno-dev)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(icp src/icp.cpp)
ament_target_dependencies(icp rclcpp sensor_msgs pcl_conversions PCL tf2 tf2_ros)

target_include_directories(icp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(icp PUBLIC c_std_99 cxx_std_17)

install(TARGETS icp
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()