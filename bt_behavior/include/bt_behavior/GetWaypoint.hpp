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

#ifndef BT_BEHAVIOR__GETWAYPOINT_HPP_
#define BT_BEHAVIOR__GETWAYPOINT_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/msg/costmap.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace bt_behavior
{

class GetWaypoint : public BT::ActionNodeBase
{
public:
  explicit GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("waypoint"),
        BT::InputPort<std::vector<std::vector<double>>>("waypoints_vector")
      });
  }
  void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  int if_obstacle(geometry_msgs::msg::Pose point);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
  int counter;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_;
};

}  // namespace bt_behavior

#endif  // BT_BEHAVIOR__GETWAYPOINT_HPP_
