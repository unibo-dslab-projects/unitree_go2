cmake_minimum_required(VERSION 3.5)
project(go2_mapping_bringup)

find_package(ament_cmake REQUIRED)

# No code: only the launch and config files, installed where
# get_package_share_directory() finds them.
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
