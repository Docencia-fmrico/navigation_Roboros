// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <vector>
#include <memory>

#include "bt_behavior/Move.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_behavior
{

using std::placeholders::_1;
using namespace std::chrono_literals;

Move::Move(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: bt_behavior::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
    conf)
{
}

void
Move::on_tick()
{
  start_time_ = node_->now();
  geometry_msgs::msg::PoseStamped goal;

  getInput("goal", goal);
  goal.header.stamp = node_->now();
  goal.header.frame_id = "/odom";
  std::cout << "Moving to waypoint***" << goal.header.frame_id << " " << std::endl;

  goal_.pose = goal;
}

void Move::on_wait_for_result()
{
  auto elapsed = node_->now() - start_time_;

  if (elapsed < 2s) {
    std::cout << "hi" << std::endl;
  } else {
    std::cout << "FINISH" << std::endl;
    halt();
  }
}

BT::NodeStatus
Move::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Suceeded");

  return BT::NodeStatus::SUCCESS;
}


}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<bt_behavior::Move>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<bt_behavior::Move>(
    "Move", builder);
}
