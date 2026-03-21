// Copyright (c) 2024 Black Coffee Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bt_navigator_live/navigators/monitored_navigate_to_pose.hpp"

namespace bt_navigator_live
{

bool MonitoredNavigateToPoseNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  // Call parent configure first to setup the action server and BT
  if (!NavigateToPoseNavigator::configure(parent_node, odom_smoother)) {
    return false;
  }

  auto node = parent_node.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Unable to lock node");
    return false;
  }

  // Declare Groot monitoring parameters
  if (!node->has_parameter("enable_groot_monitoring")) {
    node->declare_parameter("enable_groot_monitoring", false);
  }
  if (!node->has_parameter("groot_full_bt_topic")) {
    node->declare_parameter("groot_full_bt_topic", std::string("/bt_navigator/full_bt"));
  }
  if (!node->has_parameter("groot_updates_topic")) {
    node->declare_parameter("groot_updates_topic", std::string("/bt_navigator/bt_updates"));
  }

  // Get parameter values
  enable_groot_monitoring_ = node->get_parameter("enable_groot_monitoring").as_bool();
  full_bt_topic_ = node->get_parameter("groot_full_bt_topic").as_string();
  updates_topic_ = node->get_parameter("groot_updates_topic").as_string();

  if (enable_groot_monitoring_) {
    RCLCPP_INFO(
      logger_, 
      "Groot monitoring enabled for %s on topics: %s, %s",
      getName().c_str(), full_bt_topic_.c_str(), updates_topic_.c_str());
  } else {
    RCLCPP_INFO(logger_, "Groot monitoring disabled for %s", getName().c_str());
  }

  return true;
}

bool MonitoredNavigateToPoseNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  // Call parent to load the BT
  if (!NavigateToPoseNavigator::goalReceived(goal)) {
    return false;
  }

  // If monitoring is enabled and BT has been loaded, attach logger
  if (enable_groot_monitoring_) {
    try {
      // Get the tree from the action server
      const BT::Tree& tree = bt_action_server_->getTree();
      
      // Create or recreate the logger for this tree
      auto node = bt_action_server_->getBlackboard()->get<rclcpp::Node::SharedPtr>("node");
      groot_logger_ = std::make_unique<bt_monitor::ROS2TopicLogger>(
        node,
        tree,
        full_bt_topic_,
        updates_topic_);
      
      RCLCPP_INFO(
        logger_,
        "Groot logger attached to behavior tree for %s", getName().c_str());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        logger_,
        "Failed to attach Groot logger: %s", e.what());
      // Don't fail the goal if monitoring fails
    }
  }

  return true;
}

bool MonitoredNavigateToPoseNavigator::cleanup()
{
  // Clean up the Groot logger
  groot_logger_.reset();
  
  // Call parent cleanup
  return NavigateToPoseNavigator::cleanup();
}

} // namespace bt_navigator_live
