// Copyright 2021 Intelligent Robotics Lab
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

#include "bt_behavior/GetWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bt_behavior
{

using namespace std::chrono_literals;

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

void
GetWaypoint::halt()
{
  std::cout << "GetWaypoint halt" << std::endl;
}

BT::NodeStatus
GetWaypoint::tick()
{
  // Deberia hacer un getOutput id waypoint, aqui se rellena un punto a mano para probar
  geometry_msgs::msg::PoseStamped wp;
  wp.pose.position.x = 0.97;
  wp.pose.position.y = -0.65;
  wp.pose.position.z = 0;
  wp.pose.orientation.x = 0;
  wp.pose.orientation.y = 0;
  wp.pose.orientation.z = 0;
  wp.pose.orientation.w = 1;
  setOutput("waypoint", wp);


  std::cout << "I've just sent the waypoint" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetWaypoint>("GetWaypoint");
}
