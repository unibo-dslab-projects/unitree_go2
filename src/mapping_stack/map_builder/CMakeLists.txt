cmake_minimum_required(VERSION 3.5)
project(map_builder)

# C++17: std::filesystem, std::optional, unordered_map::try_emplace.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(go2_mapping_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# The core: plain C++, no ROS, no PCL. The node, the tool and any test link it,
# so nothing is compiled twice.
add_library(map_builder_core STATIC
    src/voxel_map.cpp
    src/trajectory.cpp
)
target_include_directories(map_builder_core PUBLIC include)

# Saving to disk: needs PCL for the PCD writer.
add_library(map_builder_io STATIC
    src/map_io.cpp
)
target_include_directories(map_builder_io PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(map_builder_io map_builder_core ${PCL_LIBRARIES})

# The ROS node.
add_executable(map_builder
    src/main.cpp
    src/map_builder_node.cpp
    src/cloud_msg.cpp
)
target_link_libraries(map_builder map_builder_io)
# to add go2_mapping_msgs at the end in case
ament_target_dependencies(map_builder
    rclcpp builtin_interfaces std_msgs sensor_msgs nav_msgs geometry_msgs std_srvs)

# Offline check of the voxel map on a PCD file, no ROS.
add_executable(voxel_count tools/voxel_count.cpp)
target_include_directories(voxel_count PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(voxel_count map_builder_core ${PCL_LIBRARIES})

install(TARGETS map_builder voxel_count DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # A test links the core instead of compiling its sources again, e.g.:
  # ament_add_gtest(test_voxel_map test/voxel_map.cpp)
  # target_link_libraries(test_voxel_map map_builder_core)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
