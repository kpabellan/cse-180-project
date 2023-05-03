/*
Copyright 2023 Stefano Carpin
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <rclcpp/rclcpp.hpp>
#include <final-project/navigation.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp> // to send floating point numbers
#include <cmath>
#include <iostream>

// callback function called when a laser scan is received
void processScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::cout << "Laser scan received:\n";
  double angle = msg->angle_min;

  for (const auto &range : msg->ranges)
  {
    double x = range * cos(angle);
    std::cout << "Range: " << x << "\n";
    angle += msg->angle_increment;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);         // initialize ROS
  Navigator navigator(true, false); // create node with debug info but not verbose

  rclcpp::Node::SharedPtr nodeh;
  nodeh = rclcpp::Node::make_shared("pubsub"); // create node

  // create subscriber and register callback function
  auto sub = nodeh->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, &processScan);

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);

  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = 2;
  goal_pos->position.y = -0.5;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = 2;
  goal_pos->position.y = 0.5;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = -2;
  goal_pos->position.y = 0.5;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
    
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = -0.5;
  goal_pos->position.y = 2;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = -0.5;
  goal_pos->position.y = -2;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = 0.5;
  goal_pos->position.y = -2;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = 0.5;
  goal_pos->position.y = 2;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = 0.5;
  goal_pos->position.y = -2;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  // --------------------------------------------------------------------------------------------

  goal_pos->position.x = -2;
  goal_pos->position.y = -0.5;
  goal_pos->orientation.w = 0.5;
  navigator.GoToPose(goal_pos);
  while (!navigator.IsTaskComplete())
  {
    rclcpp::spin_some(nodeh);
  }

  rclcpp::shutdown(); // shutdown ROS
  return 0;
}