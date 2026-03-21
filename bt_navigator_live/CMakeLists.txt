cmake_minimum_required(VERSION 3.5)
project(bt_navigator_live)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(bt_monitor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  behaviortree_cpp_v3
  nav2_bt_navigator
  nav2_behavior_tree
  nav2_util
  nav2_core
  nav2_msgs
  bt_monitor
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Include directories
include_directories(
  include
)

# Monitored navigators library
add_library(monitored_navigators SHARED
  src/navigators/monitored_navigate_to_pose.cpp
  src/navigators/monitored_navigate_through_poses.cpp
)
ament_target_dependencies(monitored_navigators ${dependencies})

# Monitored BT Navigator executable
add_executable(monitored_bt_navigator
  src/monitored_bt_navigator.cpp
)
ament_target_dependencies(monitored_bt_navigator ${dependencies})
target_link_libraries(monitored_bt_navigator monitored_navigators)

# Install
install(TARGETS monitored_navigators
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS monitored_bt_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export information
ament_export_include_directories(include)
ament_export_libraries(monitored_navigators)
ament_export_dependencies(${dependencies})

ament_package()
