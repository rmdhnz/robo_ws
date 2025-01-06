cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0074 NEW)
project(nexus)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(main_node src/main.cpp)
ament_target_dependencies(main_node rclcpp sensor_msgs std_msgs nav_msgs)

add_executable(icp_node src/lidar/icp_node.cpp)
ament_target_dependencies(icp_node rclcpp sensor_msgs std_msgs laser_geometry pcl_conversions PCL)

install(TARGETS
  main_node
  icp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()