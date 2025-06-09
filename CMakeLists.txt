cmake_minimum_required(VERSION 3.8)
project(navigation_filter)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Header include paths
include_directories(
  navigation_filter_library/include
)

# Library
add_library(nav_filter_lib
  navigation_filter_library/src/nav_filter.cpp
  navigation_filter_library/src/imu_model.cpp
  navigation_filter_library/src/twist_model.cpp
  navigation_filter_library/src/pose_model.cpp
  navigation_filter_library/src/sensor_model.cpp
  navigation_filter_library/src/error_state.cpp
  navigation_filter_library/src/proccess_model.cpp
)
target_link_libraries(nav_filter_lib Eigen3::Eigen)
ament_target_dependencies(nav_filter_lib rclcpp)

# Executable Node
add_executable(nav_filter ros2/nav_filter_node.cpp)
target_link_libraries(nav_filter nav_filter_lib)
target_include_directories(nav_filter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/navigation_filter_library/include
)
ament_target_dependencies(nav_filter
  rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs
  rclcpp_lifecycle tf2_ros
)

# Installation
install(TARGETS
  nav_filter_lib
  nav_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY navigation_filter_library/include/
  DESTINATION include/
)

# Linting (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
