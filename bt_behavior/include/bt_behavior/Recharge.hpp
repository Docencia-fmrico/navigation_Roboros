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

#ifndef BT_BEHAVIOR__RECHARGE_HPP_
#define BT_BEHAVIOR__RECHARGE_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "sensor_msgs/msg/battery_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_behavior
{

class Recharge : public BT::ActionNodeBase
{
public:
  explicit Recharge(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();
  void callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  bool Batterycharge;
private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batterysub_;

};

}  // namespace bt_behavior

#endif  // BT_BEHAVIOR__RECHARGE_HPP_
