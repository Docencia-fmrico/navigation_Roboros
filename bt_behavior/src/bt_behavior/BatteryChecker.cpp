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
BatteryChecker::BatteryChecker(const std::string & name){
  // no se cual sub es
  batterysub_ = create_subscription<kobuki_msgs::PowerSystemEvent>(
    "?", 10, std::bind(&BatteryChecker::callback, this, _1));
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

void callback(const kobuki_msgs::PowerSystemEvent::SharedPtr msg){
  // BATTERY_LOW= 4 or BATTERY_CRITICAL= 5
  if (msg == 4 || msg == 5) {
    Batterycharge = 1;
  } else {
    Batterycharge = 0;
  }
}
