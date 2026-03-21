# bt_monitor

General-purpose BehaviorTree.CPP v3 monitoring library and GUI for ROS2.

## Overview

`bt_monitor` provides real-time visualization and monitoring of behavior trees using the Groot protocol. It consists of two main components:

1. **ROS2TopicLogger** - A reusable library for publishing BT structure and status
2. **bt_monitor GUI** - Visual interface for observing BT execution

This package is **application-agnostic** and can be integrated into any ROS2 system using BehaviorTree.CPP v3.

## Features

- ✅ Works with standard BehaviorTree.CPP v3 binary (no custom builds)
- ✅ Minimal dependencies (behaviortree_cpp_v3, rclcpp, std_msgs)
- ✅ Publishes full tree structure (latched topic for late subscribers)
- ✅ Real-time status updates via ROS2 topics
- ✅ Qt-based GUI for visualization
- ✅ Groot protocol compatible

## Topics Published

- `/full_bt` (std_msgs/ByteMultiArray) - Full tree structure (latched)
- `/bt_updates` (std_msgs/ByteMultiArray) - Status updates

Topic names are configurable via constructor parameters.

## Usage

### As a Library (Integrating into Your Application)

```cpp
#include "bt_monitor/ros2_topic_logger.hpp"

// In your code where you create your behavior tree:
BT::Tree tree = factory.createTreeFromFile("your_tree.xml");

// Create the logger
auto node = rclcpp::Node::make_shared("your_node");
bt_monitor::ROS2TopicLogger logger(node, tree);

// Execute your tree normally - the logger will publish updates automatically
while (rclcpp::ok()) {
  tree.tickRoot();
  rclcpp::spin_some(node);
}
```

### CMakeLists.txt Integration

```cmake
find_package(bt_monitor REQUIRED)

add_executable(your_executable src/main.cpp)
ament_target_dependencies(your_executable
  rclcpp
  behaviortree_cpp_v3
  bt_monitor
)
```

### package.xml Integration

```xml
<depend>bt_monitor</depend>
```

### Running the GUI

```bash
ros2 run bt_monitor bt_monitor
```

The GUI will automatically connect to the topics and display any behavior tree that publishes to them.

### Testing with Example

Run the included test example:

```bash
ros2 launch bt_monitor example_monitor.launch.py
```

This will:
1. Start a simple behavior tree with the logger attached
2. Launch the monitoring GUI

## API Reference

### ROS2TopicLogger Constructor

```cpp
ROS2TopicLogger(
  rclcpp::Node::SharedPtr node,
  const BT::Tree& tree,
  const std::string& full_tree_topic = "/full_bt",
  const std::string& updates_topic = "/bt_updates",
  unsigned max_msg_per_second = 25
);
```

**Parameters:**
- `node` - ROS2 node for publishing
- `tree` - The behavior tree to monitor
- `full_tree_topic` - Topic for publishing tree structure (default: "/full_bt")
- `updates_topic` - Topic for status updates (default: "/bt_updates")
- `max_msg_per_second` - Rate limiting for updates (default: 25 Hz)

### Member Functions

- `void setBT(const BT::Tree& tree)` - Switch to monitoring a different tree
- `void flush()` - Manually trigger update publication (called automatically)

## Architecture

```
Your BT Application
    ↓ creates
  BT::Tree
    ↓ monitors
  ROS2TopicLogger
    ↓ publishes to
  /full_bt (latched)
  /bt_updates (real-time)
    ↓ subscribed by
  bt_monitor GUI
```

## Building from Source

```bash
cd ~/ros2_ws/src
git clone <repo-url>
cd ~/ros2_ws
colcon build --packages-select bt_monitor
```

## Dependencies

- ROS2 (tested on Humble)
- behaviortree_cpp_v3
- Qt5 (for GUI only)
- rclcpp
- std_msgs

## License

Apache 2.0

## Related Packages

- `bt_navigator_live` - Nav2 integration using this library
