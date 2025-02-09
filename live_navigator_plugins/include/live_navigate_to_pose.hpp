// Copyright (c) 2021 Samsung Research
// Copyright (c) 2024 Arthur Gomes
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

#pragma once
#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_bt_navigator/navigator.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/odometry_utils.hpp"

namespace live_navigator_plugins
{
    class LiveNavigateToPose
        : public nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateToPose>
    {
    public:
        using ActionT = nav2_msgs::action::NavigateToPose;

        NavigateToPoseNavigator() : Navigator() {} // todo

        bool configure(
            rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
            std::shared_ptr<nav2_util::OdomSmoother> odom_smoother) override;

        bool cleanup() override;

        void onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

        std::string getName() override {return std::string("live_navigate_to_pose");}

        std::string getBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

    protected:

        bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

        void onLoop() override;

        void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

        void goalCompleted(
            typename ActionT::Result::SharedPtr result,
            const nav2_behavior_tree::BtStatus final_bt_status) override {};

        void initializeGoalPose(ActionT::Goal::ConstSharedPtr goal);

        rclcpp::Time start_time_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
        rclcpp_action::Client<ActionT>::SharedPtr self_client_;

        std::string goal_blackboard_id_;
        std::string path_blackboard_id_;

        std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
    };
}