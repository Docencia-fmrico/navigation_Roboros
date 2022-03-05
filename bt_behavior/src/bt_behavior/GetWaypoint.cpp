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
#include "nav_msgs/msg/occupancy_grid.hpp"
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
  costmap_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", 10,std::bind(&GetWaypoint::callback, this, _1));
  //occupancy_grid_->data[0] = -2;
}

void GetWaypoint::callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  //std::cout << "fill" << std::endl;
  occupancy_grid_ = msg;
}


int GetWaypoint::if_obstacle(geometry_msgs::msg::Pose point){
  
  //int oc_point = occupancy_grid_->info.width * point.position.y + point.position.x;
  if (occupancy_grid_ == NULL)
    return -1;
  if (point.position.x < occupancy_grid_->info.origin.position.x || point.position.y < occupancy_grid_->info.origin.position.y){
    std::cout << "Out of boundaries" << std::endl;
    return 1;
  }
  int point_map_x = ((point.position.x - occupancy_grid_->info.origin.position.x) * (1.0/occupancy_grid_->info.resolution ));
  int point_map_y = ((point.position.y - occupancy_grid_->info.origin.position.y) * (1.0/occupancy_grid_->info.resolution ));
  int point_map = (occupancy_grid_->info.width * point_map_y) + point_map_x;
  std::cout << point_map_x << " : " << point_map_y << std::endl;
  if ( occupancy_grid_->data[point_map] == 0){
    std::cout << "free" << std::endl;
    return 0;
  } 
  std::cout << "obstacle" << std::endl;
  return 1;
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

  wp.pose.position.x = waypoints_vector[counter][0];
  wp.pose.position.y = waypoints_vector[counter][1];
  std::cout << waypoints_vector.size() << std::endl;
  if ( counter+1 >= waypoints_vector.size() )
    counter = 0;

  if( if_obstacle(wp.pose) == -1)
    return BT::NodeStatus::RUNNING;
  if( if_obstacle(wp.pose) == 1){
    counter++;
    return BT::NodeStatus::RUNNING;
  }
  wp.pose.position.z = 0;
  wp.pose.orientation.x = 0;
  wp.pose.orientation.y = 0;
  wp.pose.orientation.z = 0;
  wp.pose.orientation.w = 1;
  setOutput("waypoint", wp);

  std::cout << "I've just sent the waypoint " << counter <<std::endl;
  counter++;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_behavior::GetWaypoint>("GetWaypoint");
}
