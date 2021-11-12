// Copyright 2015 Open Source Robotics Foundation, Inc.
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

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"

struct Bridge1to2HandlesAndMessageTypes
{
  ros1_bridge::Bridge1to2Handles bridge_handles;
  std::string ros1_type_name;
  std::string ros2_type_name;
};

struct Bridge2to1HandlesAndMessageTypes
{
  ros1_bridge::Bridge2to1Handles bridge_handles;
  std::string ros1_type_name;
  std::string ros2_type_name;
};

int main(int argc, char * argv[])
{
  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  std::map<std::string, Bridge1to2HandlesAndMessageTypes> bridges_1to2;
  std::map<std::string, Bridge2to1HandlesAndMessageTypes> bridges_2to1;

  // // bridge one example topic
  // std::string topic_name0 = "tf";
  // std::string ros1_type_name0 = "tf2_msgs/TFMessage";
  // std::string ros2_type_name0 = "tf2_msgs/msg/TFMessage";
  // size_t queue_size = 10;
  // auto handles = ros1_bridge::create_bidirectional_bridge(
  //   ros1_node, ros2_node, ros1_type_name0, ros2_type_name0, topic_name0, queue_size);
  
  // // Bridge a second topic
  // topic_name0 = "tf_static";
  // ros1_type_name0 = "tf2_msgs/TFMessage";
  // ros2_type_name0 = "tf2_msgs/msg/TFMessage";
  // auto handles2 = ros1_bridge::create_bidirectional_bridge(
  //   ros1_node, ros2_node, ros1_type_name0, ros2_type_name0, topic_name0, queue_size);
  

  // 1 to 2 bridges
  // **************

  std::vector<std::string> topics_1to2;
  std::vector<std::string> ros2_types_1to2;
  std::vector<std::string> ros1_types_1to2;

  topics_1to2.push_back(std::string("tf"));
  ros2_types_1to2.push_back(std::string("tf2_msgs/msg/TFMessage"));
  ros1_types_1to2.push_back(std::string("tf2_msgs/TFMessage"));

  topics_1to2.push_back(std::string("tf_static"));
  ros2_types_1to2.push_back(std::string("tf2_msgs/msg/TFMessage"));
  ros1_types_1to2.push_back(std::string("tf2_msgs/TFMessage"));


  // create 1to2 bridges
  for (int i = 0; i < topics_1to2.size(); i++)
  {

    // check if 1to2 bridge for the topic exists
    if (bridges_1to2.find(topics_1to2[i]) != bridges_1to2.end()) {
      auto bridge = bridges_1to2.find(topics_1to2[i])->second;
      if (bridge.ros1_type_name == ros1_types_1to2[i] && bridge.ros2_type_name == ros2_types_1to2[i]) {
        // skip if bridge with correct types is already in place
        continue;
      }
      // remove existing bridge with previous types
      bridges_1to2.erase(topics_1to2[i]);
      printf("replace 1to2 bridge for topic '%s'\n", topics_1to2[i].c_str());
    }

    Bridge1to2HandlesAndMessageTypes bridge;
    bridge.ros1_type_name = ros1_types_1to2[i];
    bridge.ros2_type_name = ros2_types_1to2[i];

    try {
      bridge.bridge_handles = ros1_bridge::create_bridge_from_1_to_2(
        ros1_node, ros2_node,
        bridge.ros1_type_name, topics_1to2[i], 10,
        bridge.ros2_type_name, topics_1to2[i], 10);
    } catch (std::runtime_error & e) {
      fprintf(
        stderr,
        "failed to create 1to2 bridge for topic '%s' "
        "with ROS 1 type '%s' and ROS 2 type '%s': %s\n",
        topics_1to2[i].c_str(), bridge.ros1_type_name.c_str(), bridge.ros2_type_name.c_str(), e.what());
      if (std::string(e.what()).find("No template specialization") != std::string::npos) {
        fprintf(stderr, "check the list of supported pairs with the `--print-pairs` option\n");
      }
      continue;
    }

    bridges_1to2[topics_1to2[i]] = bridge;
    printf(
      "created 1to2 bridge for topic '%s' with ROS 1 type '%s' and ROS 2 type '%s'\n",
      topics_1to2[i].c_str(), bridge.ros1_type_name.c_str(), bridge.ros2_type_name.c_str());
  }


  // 2 to 1 bridges
  // **************
  
  std::vector<std::string> topics_2to1;
  std::vector<std::string> ros2_types_2to1;
  std::vector<std::string> ros1_types_2to1;

  topics_2to1.push_back(std::string("rosout"));
  ros2_types_2to1.push_back(std::string("rcl_interfaces/msg/Log"));
  ros1_types_2to1.push_back(std::string("rosgraph_msgs/Log"));

  // Create 2to1 bridges
  for (int i = 0; i < topics_2to1.size(); i++)
  {

    // check if 2to1 bridge for the topic exists
    if (bridges_2to1.find(topics_2to1[i]) != bridges_2to1.end()) {
      auto bridge = bridges_2to1.find(topics_2to1[i])->second;
      if ((bridge.ros1_type_name == ros1_types_2to1[i] || bridge.ros1_type_name == "") &&
        bridge.ros2_type_name == ros2_types_2to1[i])
      {
        // skip if bridge with correct types is already in place
        continue;
      }
      // remove existing bridge with previous types
      bridges_2to1.erase(topics_2to1[i]);
      printf("replace 2to1 bridge for topic '%s'\n", topics_2to1[i].c_str());
    }

    Bridge2to1HandlesAndMessageTypes bridge;
    bridge.ros1_type_name = ros1_types_2to1[i];
    bridge.ros2_type_name = ros2_types_2to1[i];
    try {
      bridge.bridge_handles = ros1_bridge::create_bridge_from_2_to_1(
        ros2_node, ros1_node,
        bridge.ros2_type_name, topics_2to1[i], 10,
        bridge.ros1_type_name, topics_2to1[i], 10);
    } catch (std::runtime_error & e) {
      fprintf(
        stderr,
        "failed to create 2to1 bridge for topic '%s' "
        "with ROS 2 type '%s' and ROS 1 type '%s': %s\n",
        topics_2to1[i].c_str(), bridge.ros2_type_name.c_str(), bridge.ros1_type_name.c_str(), e.what());
      if (std::string(e.what()).find("No template specialization") != std::string::npos) {
        fprintf(stderr, "check the list of supported pairs with the `--print-pairs` option\n");
      }
      continue;
    }
    bridges_2to1[topics_2to1[i]] = bridge;
    printf("created 2to1 bridge for topic '%s' with ROS 2 type '%s' and ROS 1 type '%s'\n",
      topics_2to1[i].c_str(), bridge.ros2_type_name.c_str(), bridge.ros1_type_name.c_str());
  }

  // ROS 1 asynchronous spinner
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  // ROS 2 spinning loop
  rclcpp::executors::SingleThreadedExecutor executor;
  while (ros1_node.ok() && rclcpp::ok()) {
    executor.spin_node_once(ros2_node, std::chrono::milliseconds(1000));
  }

  return 0;
}
