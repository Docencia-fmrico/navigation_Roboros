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
#include <vector>
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("patrolling_node");

  node->declare_parameter("waypoints");
  rclcpp::Parameter param("waypoints", std::vector<std::string>({}));
  node->get_parameter("waypoints", param);
  std::vector<std::string> waypoints = param.as_string_array();
  std::cout << waypoints.size() << std::endl;
  RCLCPP_INFO(node->get_logger(), "%d", waypoints.size());
  std::vector<std::vector<long int>> wps;
  for (int i = 0; i < 2; i++) {
    std::string wp ="wp" + std::to_string(i);
    rclcpp::Parameter param(wp, std::vector<long int>({}));
    node->get_parameter(wp, param);
    //wps.push_back(param.as_integer_array());
    std::cout << param.as_integer_array()[0];
  }
  
  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("br2_move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("br2_patrol_bt_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_behavior");
  std::string xml_file = pkgpath + "/behavior_tree_xml/simple.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
