/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <chrono>
#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waypoint_publisher");
  auto trajectory_pub =
      node->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  RCLCPP_INFO(node->get_logger(), "Started waypoint_publisher.");

  const auto args = rclcpp::remove_ros_arguments(argc, argv);

  double delay;

  if (args.size() == 5) {
    delay = 1.0;
  } else if (args.size() == 6) {
    delay = std::stof(args.at(5));
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "Usage: waypoint_publisher <x> <y> <z> <yaw_deg> [<delay>]");
    rclcpp::shutdown();
    return -1;
  }

  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::msg::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->get_clock()->now();

  Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(2)),
                                   std::stof(args.at(3)));

  double desired_yaw = std::stof(args.at(4)) * DEG_2_RAD;

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);

  // Wait for some time to create the ros publisher.
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delay)));

  while (trajectory_pub->get_subscription_count() == 0 && rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(),
                "There is no subscriber available, trying again in 1 second.");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  RCLCPP_INFO(node->get_logger(),
           "Publishing waypoint on namespace %s: [%f, %f, %f].",
           node->get_namespace(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());

  trajectory_pub->publish(trajectory_msg);

  rclcpp::spin_some(node);
  rclcpp::shutdown();

  return 0;
}
