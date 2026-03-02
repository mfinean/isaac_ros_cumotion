// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "isaac_ros_cumotion_moveit/cumotion_move_group_client.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/motion_plan_response.hpp"
#include "moveit_msgs/msg/planning_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nvidia
{
namespace isaac
{
namespace manipulation
{

namespace
{

constexpr unsigned kGetGoalWaitIntervalInMs = 10;

}  // namespace

CumotionMoveGroupClient::CumotionMoveGroupClient(const rclcpp::Node::SharedPtr & node)
: result_ready(false),
  success(false),
  get_goal_handle_(false),
  get_result_handle_(false),
  node_(node),
  client_cb_group_(node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
{
  client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
    node_,
    "cumotion/move_group",
    client_cb_group_);

  send_goal_options_ = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

  send_goal_options_.goal_response_callback = std::bind(
    &CumotionMoveGroupClient::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options_.feedback_callback = std::bind(
    &CumotionMoveGroupClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

  // Subscribe to the result topic published by the cuMotion Python planner.
  result_sub_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
    "cumotion/planning_result",
    rclcpp::QoS(1).reliable(),
    std::bind(&CumotionMoveGroupClient::resultTopicCallback, this, std::placeholders::_1));
}

void CumotionMoveGroupClient::updateGoal(
  const planning_scene::PlanningSceneConstPtr & planning_scene,
  const planning_interface::MotionPlanRequest & req)
{
  planning_request_ = req;
  planning_scene->getPlanningSceneMsg(planning_scene_);
}

bool CumotionMoveGroupClient::sendGoal()
{
  result_ready = false;
  success = false;
  get_result_handle_ = true;  // Arms the topic callback for exactly one result

  moveit_msgs::msg::PlanningOptions plan_options;
  plan_options.planning_scene_diff = planning_scene_;

  if (!client_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = moveit_msgs::action::MoveGroup::Goal();

  goal_msg.planning_options = plan_options;
  goal_msg.request = planning_request_;
  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto goal_handle_future = client_->async_send_goal(goal_msg, send_goal_options_);
  goal_h_ = goal_handle_future;
  get_goal_handle_ = true;
  return true;
}

void CumotionMoveGroupClient::getGoal()
{
  if (get_goal_handle_) {
    if (goal_h_.wait_for(std::chrono::milliseconds(kGetGoalWaitIntervalInMs)) !=
      std::future_status::ready)
    {
      return;
    }

    GoalHandle::SharedPtr goal_handle = goal_h_.get();

    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      success = false;
      result_ready = true;
    }
    get_goal_handle_ = false;
  }
}

void CumotionMoveGroupClient::goalResponseCallback(const GoalHandle::SharedPtr & future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    result_ready = true;
    success = false;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void CumotionMoveGroupClient::feedbackCallback(
  GoalHandle::SharedPtr,
  const std::shared_ptr<const moveit_msgs::action::MoveGroup::Feedback> feedback)
{
  std::string status = feedback->state;
  RCLCPP_INFO(node_->get_logger(), "Checking status");
  RCLCPP_INFO(node_->get_logger(), status.c_str());
}

void CumotionMoveGroupClient::resultCallback(const GoalHandle::WrappedResult & result)
{
  (void)result;
}

void CumotionMoveGroupClient::resultTopicCallback(
  const moveit_msgs::msg::MotionPlanResponse::SharedPtr msg)
{
  // Atomically claim this result. get_result_handle_ is set to true in
  // sendGoal() and flipped to false by the first callback to arrive.
  // All subsequent callbacks (CycloneDDS retransmissions or executor
  // thread races) see false and return immediately.
  bool expected = true;
  if (!get_result_handle_.compare_exchange_strong(expected, false)) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(),
    "Received result via topic: error_code.val=%d, trajectory points=%zu",
    msg->error_code.val,
    msg->trajectory.joint_trajectory.points.size());

  plan_response.error_code = msg->error_code;

  if (msg->error_code.val == 1) {  // MoveItErrorCodes::SUCCESS
    plan_response.trajectory_start = msg->trajectory_start;
    plan_response.group_name = msg->group_name;
    plan_response.trajectory.resize(1);
    plan_response.trajectory[0] = msg->trajectory;
    plan_response.processing_time = {msg->planning_time};
    success = true;
  } else {
    success = false;
  }

  // Set result_ready LAST — all data must be populated before this.
  result_ready = true;
}

}  // namespace manipulation
}  // namespace isaac
}  // namespace nvidia
