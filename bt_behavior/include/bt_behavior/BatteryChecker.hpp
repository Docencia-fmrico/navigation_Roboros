// Copyright (c) 2018 Intel Corporation
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


#ifndef BT_BEHAVIOR__BATTERYCHECKER_HPP_
#define BT_BEHAVIOR__BATTERYCHECKER_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "bt_behavior/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_behavior
{

class BatteryChecker : public BT::SyncActionNode
{
public:
  explicit BatteryChecker(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick() override;
  void callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);



private:
  bool Batterycharge;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr batterysub_;
};

}  // namespace bt_behavior

#endif  // BT_BEHAVIOR__BATTERYCHECKER_HPP_
