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

#ifndef BT_NAVIGATOR_LIVE__MONITORED_BT_NAVIGATOR_HPP_
#define BT_NAVIGATOR_LIVE__MONITORED_BT_NAVIGATOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#include "bt_navigator_live/navigators/monitored_navigate_to_pose.hpp"
#include "bt_navigator_live/navigators/monitored_navigate_through_poses.hpp"
#include "nav2_bt_navigator/navigator.hpp"

namespace bt_navigator_live
{

/**
 * @class MonitoredBtNavigator
 * @brief BT Navigator with integrated Groot live monitoring support
 * 
 * This is a drop-in replacement for nav2_bt_navigator that adds optional
 * live monitoring capabilities via Groot-compatible ROS2 topics.
 */
class MonitoredBtNavigator : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief Constructor
   * @param options Node options for configuration
   */
  explicit MonitoredBtNavigator(rclcpp::NodeOptions options = rclcpp::NodeOptions());
  
  /**
   * @brief Destructor
   */
  ~MonitoredBtNavigator();

protected:
  /**
   * @brief Configure lifecycle callback
   * Initializes navigators with monitoring support
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  
  /**
   * @brief Activate lifecycle callback
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  
  /**
   * @brief Deactivate lifecycle callback
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  
  /**
   * @brief Cleanup lifecycle callback
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  
  /**
   * @brief Shutdown lifecycle callback
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Monitored navigators that extend Nav2's navigators
  std::unique_ptr<nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateToPose>> pose_navigator_;
  std::unique_ptr<nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateThroughPoses>> poses_navigator_;
  nav2_bt_navigator::NavigatorMuxer plugin_muxer_;

  // Odometry smoother object
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  // Metrics for feedback
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace bt_navigator_live

#endif  // BT_NAVIGATOR_LIVE__MONITORED_BT_NAVIGATOR_HPP_
