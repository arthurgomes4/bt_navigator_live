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

#ifndef BT_NAVIGATOR_LIVE__NAVIGATORS__MONITORED_NAVIGATE_THROUGH_POSES_HPP_
#define BT_NAVIGATOR_LIVE__NAVIGATORS__MONITORED_NAVIGATE_THROUGH_POSES_HPP_

#include <string>
#include <memory>

#include "nav2_bt_navigator/navigators/navigate_through_poses.hpp"
#include "bt_monitor/ros2_topic_logger.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_navigator_live
{

/**
 * @class MonitoredNavigateThroughPosesNavigator
 * @brief Extended NavigateThroughPoses navigator with live Groot monitoring support
 */
class MonitoredNavigateThroughPosesNavigator 
  : public nav2_bt_navigator::NavigateThroughPosesNavigator
{
public:
  using ActionT = nav2_msgs::action::NavigateThroughPoses;

  /**
   * @brief Constructor
   */
  MonitoredNavigateThroughPosesNavigator()
  : NavigateThroughPosesNavigator() {}

  /**
   * @brief Configure the navigator with Groot monitoring
   * @param node Weakptr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot's speed
   * @return bool Success
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief Cleanup the navigator and monitoring resources
   * @return bool Success
   */
  bool cleanup() override;

protected:
  /**
   * @brief Setup Groot monitoring after BT is loaded
   * @param goal Action goal that may contain BT XML
   * @return bool Success
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  std::unique_ptr<bt_monitor::ROS2TopicLogger> groot_logger_;
  bool enable_groot_monitoring_;
  std::string full_bt_topic_;
  std::string updates_topic_;
};

} // namespace bt_navigator_live

#endif  // BT_NAVIGATOR_LIVE__NAVIGATORS__MONITORED_NAVIGATE_THROUGH_POSES_HPP_
