cmake_minimum_required(VERSION 3.5)
project(go2_preprocessor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(go2_preprocessor
    src/main.cpp
    src/convertor.cpp
    src/schemas.cpp
    src/robot_utils.cpp
)

target_include_directories(go2_preprocessor PRIVATE include)

ament_target_dependencies(go2_preprocessor rclcpp sensor_msgs)

install(TARGETS go2_preprocessor DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The test compiles convertor.cpp again because the code lives in the
  # executable, not in a library the test could link to.
  ament_add_gtest(test_convertor
    test/convertor.cpp
    src/convertor.cpp
  )
  target_include_directories(test_convertor PRIVATE include)
  ament_target_dependencies(test_convertor sensor_msgs)

  ament_add_gtest(test_schemas
    test/schemas.cpp
    src/schemas.cpp
  )
  target_include_directories(test_schemas PRIVATE include)

  ament_add_gtest(test_robot_utils
    test/robot_utils.cpp
    src/robot_utils.cpp
    src/schemas.cpp
  )
  target_include_directories(test_robot_utils PRIVATE include)

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in
  # all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
