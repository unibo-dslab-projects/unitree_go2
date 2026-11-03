cmake_minimum_required(VERSION 3.5)
project(vcortex_bridge)

# C++17: std::optional
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(go2_mapping_msgs REQUIRED)
# beast is header-only
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

add_executable(vcortex_bridge
    src/main.cpp
    src/node.cpp
    src/ingest_client.cpp
)
target_include_directories(vcortex_bridge PRIVATE include ${Boost_INCLUDE_DIRS})
target_link_libraries(vcortex_bridge Threads::Threads)
ament_target_dependencies(vcortex_bridge rclcpp geometry_msgs sensor_msgs go2_mapping_msgs)

install(TARGETS vcortex_bridge DESTINATION lib/${PROJECT_NAME})

ament_package()
