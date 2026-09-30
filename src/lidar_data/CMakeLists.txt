cmake_minimum_required(VERSION 3.16)
project(lidar_data)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# find_package locates an installed package and loads its include paths and libraries
# into CMake variables. It does not apply them to any target yet.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cloud_inspector
  src/main.cpp
  src/cloud_inspector.cpp
)

# Project own headers: lets the code write #include "lidar_data/...".
# PRIVATE = only this target sees them.
target_include_directories(cloud_inspector PRIVATE include)

# Applies what find_package found: adds the ROS include paths (-isystem) and links the libraries.
ament_target_dependencies(cloud_inspector rclcpp sensor_msgs)

install(TARGETS cloud_inspector DESTINATION lib/${PROJECT_NAME})

ament_package()
