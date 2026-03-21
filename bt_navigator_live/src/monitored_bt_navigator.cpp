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

#include "bt_navigator_live/monitored_bt_navigator.hpp"

#include <memory>
#include <string>
#include <vector>

namespace bt_navigator_live
{

MonitoredBtNavigator::MonitoredBtNavigator(rclcpp::NodeOptions options)
: nav2_util::LifecycleNode("bt_navigator", "",
    options.automatically_declare_parameters_from_overrides(true))
{
  RCLCPP_INFO(get_logger(), "Creating Monitored BT Navigator with Groot support");

  // Declare all the same parameters as the standard bt_navigator
  const std::vector<std::string> plugin_libs = {
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_compute_and_track_route_bt_node",
    "nav2_compute_route_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_assisted_teleop_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node",
    "nav2_back_up_cancel_bt_node",
    "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node"
  };

  declare_parameter_if_not_declared(
    this, "plugin_lib_names", rclcpp::ParameterValue(plugin_libs));
  declare_parameter_if_not_declared(
    this, "transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    this, "global_frame", rclcpp::ParameterValue(std::string("map")));
  declare_parameter_if_not_declared(
    this, "robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
  declare_parameter_if_not_declared(
    this, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
  
  // Additional parameters for Groot monitoring
  declare_parameter_if_not_declared(
    this, "enable_groot_monitoring", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(
    this, "groot_full_bt_topic", rclcpp::ParameterValue(std::string("/bt_navigator/full_bt")));
  declare_parameter_if_not_declared(
    this, "groot_updates_topic", rclcpp::ParameterValue(std::string("/bt_navigator/bt_updates")));
}

MonitoredBtNavigator::~MonitoredBtNavigator()
{
}

nav2_util::CallbackReturn
MonitoredBtNavigator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring Monitored BT Navigator");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  // Create MONITORED navigators instead of standard Nav2 navigators
  pose_navigator_ = std::make_unique<MonitoredNavigateToPoseNavigator>();
  poses_navigator_ = std::make_unique<MonitoredNavigateThroughPosesNavigator>();

  nav2_bt_navigator::FeedbackUtils feedback_utils;
  feedback_utils.tf = tf_;
  feedback_utils.global_frame = global_frame_;
  feedback_utils.robot_frame = robot_frame_;
  feedback_utils.transform_tolerance = transform_tolerance_;

  // Odometry smoother object for getting current speed
  odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(shared_from_this(), 0.3, odom_topic_);

  if (!pose_navigator_->on_configure(
      shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_, odom_smoother_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  if (!poses_navigator_->on_configure(
      shared_from_this(), plugin_lib_names, feedback_utils, &plugin_muxer_, odom_smoother_))
  {
    return nav2_util::CallbackReturn::FAILURE;
  }

  bool groot_enabled = get_parameter("enable_groot_monitoring").as_bool();
  if (groot_enabled) {
    RCLCPP_INFO(get_logger(), "Groot monitoring is ENABLED");
  } else {
    RCLCPP_INFO(get_logger(), "Groot monitoring is DISABLED (set enable_groot_monitoring:=true to enable)");
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MonitoredBtNavigator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating Monitored BT Navigator");

  if (!poses_navigator_->on_activate() || !pose_navigator_->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MonitoredBtNavigator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating Monitored BT Navigator");

  if (!poses_navigator_->on_deactivate() || !pose_navigator_->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MonitoredBtNavigator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up Monitored BT Navigator");

  pose_navigator_->on_cleanup();
  poses_navigator_->on_cleanup();
  pose_navigator_.reset();
  poses_navigator_.reset();
  tf_.reset();
  tf_listener_.reset();
  odom_smoother_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MonitoredBtNavigator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down Monitored BT Navigator");
  return nav2_util::CallbackReturn::SUCCESS;
}

} // namespace bt_navigator_live

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bt_navigator_live::MonitoredBtNavigator>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
