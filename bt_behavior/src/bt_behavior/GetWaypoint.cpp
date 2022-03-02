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
 #include "nav2_costmap_2d/costmap_2d.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_behavior
{
using std::placeholders::_1;

using namespace std::chrono_literals;

GetWaypoint::GetWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  counter = 0;
  //costmap_ = node_->create_subscription<nav2_costmap_2d::Costmap2D>(
  //  "scan_raw", 10,std::bind(&GetWaypoint::callback, this, _1));
  lasersub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan_raw", 10, std::bind(&GetWaypoint::callback, this, _1));
}

void GetWaypoint::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

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
  std::vector<std::vector<double>> waypoints_vector;
  //getInput("waypoints_vector",waypoints_vector);
  config().blackboard->get("waypoints_vector", waypoints_vector);
  std::cout << counter++ << std::endl;

  if (waypoints_vector.size() <= counter)
    counter = 0;
  
  wp.pose.position.x = waypoints_vector[counter][0];
  wp.pose.position.y = waypoints_vector[counter][1];
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
