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

#include "bt_behavior/Patrol.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "bt_behavior/Recharge.hpp"

namespace bt_behavior
{

Recharge::Recharge(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // no se cual sub es
  batterysub_ = rclcpp::create_subscription<sensor_msgs::Batterystate>(
    "?", 10, std::bind(&Recharge::callback, this, _1));
}

void
Patrol::halt()
{
  std::cout << "Recharge halt" << std::endl;
}

BT::NodeStatus
Recharge::tick()
{
  if (Batterycharge) {
    return BT::NodeStatus::SUCCESS;
  }
}

void callback(const sensor_msgs::Batterystate::SharedPtr msg){
  // uint8 POWER_SUPPLY_STATUS_FULL = 4
  if (msg->power_supply_status == 4) {
    Batterycharge = 1;
  } else {
    Batterycharge = 0;
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::Recharge>("Recharge");
}
