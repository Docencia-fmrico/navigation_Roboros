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

#include "bt_behavior/BatteryChecker.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_behavior
{

BatteryChecker::BatteryChecker(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // no se cual sub es
  batterysub_ = rclcpp::create_subscription<sensor_msgs::msg::BatteryState>(
  "?", 10, std::bind(&BatteryChecker::callback, this,std::placeholders::_1));

  Batterycharge=false;
}

void
BatteryChecker::halt()
{
  std::cout << "Recharge needed" << std::endl;
}

BT::NodeStatus
BatteryChecker::tick()
{
  if (Batterycharge) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

void BatteryChecker::callback(const sensor_msgs::msg::BatteryState::SharedPtr msg){
  // BATTERY_LOW= 4 or BATTERY_CRITICAL= 5
  if (msg->power_supply_status == 4 || msg->power_supply_status == 5) {
    Batterycharge = true;
  } else {
    Batterycharge = false;
  }
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::BatteryChecker>("BatteryChecker");
}
