cmake_minimum_required(VERSION 3.5)
project(go2_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

set(ros_dependencies
    builtin_interfaces geometry_msgs rclcpp rosbag2_cpp rosbag2_storage sensor_msgs std_msgs unitree_api unitree_go)

# The calibration itself, without ROS: motion sequence, buffer, fits, checks and report.
add_library(go2_calibration_core STATIC
    src/buffer.cpp
    src/calibrate.cpp
    src/fitting.cpp
    src/floor.cpp
    src/imu_fit.cpp
    src/report.cpp
    src/sequence.cpp
)
target_include_directories(go2_calibration_core PUBLIC include)
target_link_libraries(go2_calibration_core PUBLIC Eigen3::Eigen)

# The ROS side: message conversions and the two sources that fill the buffer.
add_library(go2_calibration_ros STATIC
    src/bag_source.cpp
    src/live_source.cpp
    src/messages.cpp
)
target_link_libraries(go2_calibration_ros go2_calibration_core)
ament_target_dependencies(go2_calibration_ros ${ros_dependencies})

add_executable(go2_calibration src/main.cpp)
target_link_libraries(go2_calibration go2_calibration_ros)
ament_target_dependencies(go2_calibration ${ros_dependencies})

install(TARGETS go2_calibration DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_go2_calibration
    test/bag_source.cpp
    test/buffer.cpp
    test/calibrate.cpp
    test/fitting.cpp
    test/floor.cpp
    test/imu_fit.cpp
    test/messages.cpp
    test/report.cpp
    test/sequence.cpp
  )
  target_link_libraries(test_go2_calibration go2_calibration_ros)
  ament_target_dependencies(test_go2_calibration ${ros_dependencies})
endif()

ament_package()
