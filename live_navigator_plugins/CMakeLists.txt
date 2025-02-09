cmake_minimum_required(VERSION 3.5)
project(live_navigator_plugins)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

set(library_name live_navigator_plugins)

set(dependencies
  behaviortree_cpp_v3
)

add_executable(test_zmq src/test_zmq.cpp)
ament_target_dependencies(test_zmq ${dependencies})

install(TARGETS test_zmq
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()