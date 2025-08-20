cmake_minimum_required(VERSION 3.22)

# Update the policy setting to avoid an error when loading the ament_cmake package
# at the current cmake version level
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

project(bt_monitor)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets Gui OpenGL Xml Svg REQUIRED)

# Set C++ standard
if(NOT CMAKE_VERSION VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 11)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

set(dependencies
  behaviortree_cpp_v3
  rclcpp
  std_msgs
)

# Qt configuration
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_definitions(${Qt5Widgets_DEFINITIONS} -DNODE_EDITOR_STATIC)
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

# Include directories
INCLUDE_DIRECTORIES(include ./QtNodeEditor/include ./)

# Add QtNodeEditor subdirectory
add_subdirectory(QtNodeEditor)

# Source files for behavior tree monitor
set(APP_CPPS
    ./bt_editor/models/BehaviorTreeNodeModel.cpp
    ./bt_editor/models/SubtreeNodeModel.cpp
    ./bt_editor/mainwindow.cpp
    ./bt_editor/editor_flowscene.cpp
    ./bt_editor/utils.cpp
    ./bt_editor/bt_editor_base.cpp
    ./bt_editor/graphic_container.cpp
    ./bt_editor/XML_utilities.cpp
    ./bt_editor/sidepanel_monitor.cpp
)

# Resource files
set(RESOURCE_FILES
    ./bt_editor/resources/icons.qrc
    ./bt_editor/resources/style.qrc
)

# UI forms
set(FORMS_UI
    ./bt_editor/about_dialog.ui
    ./bt_editor/mainwindow.ui
    ./bt_editor/sidepanel_monitor.ui
)

# Wrap UI files
QT5_WRAP_UI(FORMS_HEADERS ${FORMS_UI})

# Build libraries
add_library(behavior_tree_monitor SHARED
    ${APP_CPPS}
    ${FORMS_HEADERS}
)

add_library(ros2_topic_logger SHARED
    ./src/ros2_topic_logger.cpp
)

# Set up dependencies
SET(GROOT_DEPENDENCIES QtNodeEditor ncurses ncursesw tinfo)

# Add ament dependencies
ament_target_dependencies(behavior_tree_monitor ${dependencies})
ament_target_dependencies(ros2_topic_logger ${dependencies})

# Link libraries
target_link_libraries(behavior_tree_monitor ${GROOT_DEPENDENCIES})
target_link_libraries(ros2_topic_logger ${GROOT_DEPENDENCIES})

# Build executables
add_executable(bt_monitor ./bt_editor/main.cpp ${RESOURCE_FILES})
add_executable(test_ros2_topic_logger ./src/test_ros2_topic_logger.cpp)

# Link executables
target_link_libraries(bt_monitor behavior_tree_monitor)
ament_target_dependencies(test_ros2_topic_logger
    rclcpp
    behaviortree_cpp_v3
    std_msgs
)
target_link_libraries(test_ros2_topic_logger ros2_topic_logger)

# Installation
INSTALL(TARGETS behavior_tree_monitor LIBRARY DESTINATION lib)
INSTALL(TARGETS ros2_topic_logger LIBRARY DESTINATION lib)
INSTALL(TARGETS bt_monitor RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS
    test_ros2_topic_logger
    DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
    DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Ament package configuration
ament_export_include_directories(include)
ament_export_dependencies(${GROOT_DEPENDENCIES} ${dependencies})
ament_export_libraries(behavior_tree_monitor ros2_topic_logger)
ament_package()
