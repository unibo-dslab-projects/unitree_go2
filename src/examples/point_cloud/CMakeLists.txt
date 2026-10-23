cmake_minimum_required(VERSION 3.16)
project(point_cloud)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cloud_builder
  src/main.cpp
  src/cloud_builder.cpp
  src/cloud_conversion.cpp
)

target_include_directories(cloud_builder PRIVATE include)

ament_target_dependencies(cloud_builder rclcpp sensor_msgs)

install(TARGETS cloud_builder DESTINATION lib/${PROJECT_NAME})
ament_package()
