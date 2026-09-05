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

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("hovering_example");
  // ROS 2 node parameters are the equivalent of the ROS 1 private handle.
  auto trajectory_pub =
      node->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  RCLCPP_INFO(node->get_logger(), "Started hovering example.");

  auto unpause_client = node->create_client<std_srvs::srv::Empty>(
      "/gazebo/unpause_physics");
  bool unpaused = false;
  if (unpause_client->wait_for_service(std::chrono::seconds(1))) {
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = unpause_client->async_send_request(request);
    const auto result = rclcpp::spin_until_future_complete(
        node, future, std::chrono::seconds(1));
    unpaused = result == rclcpp::FutureReturnCode::SUCCESS;
  }
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused && rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(),
                "Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (unpause_client->wait_for_service(std::chrono::seconds(1))) {
      auto request = std::make_shared<std_srvs::srv::Empty::Request>();
      auto future = unpause_client->async_send_request(request);
      const auto result = rclcpp::spin_until_future_complete(
          node, future, std::chrono::seconds(1));
      unpaused = result == rclcpp::FutureReturnCode::SUCCESS;
    }
    ++i;
  }

  if (!unpaused) {
    RCLCPP_FATAL(node->get_logger(), "Could not wake up Gazebo.");
    rclcpp::shutdown();
    return -1;
  } else {
    RCLCPP_INFO(node->get_logger(), "Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(5.0)));

  trajectory_msgs::msg::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->get_clock()->now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
  double desired_yaw = 0.0;

  // Overwrite defaults if set as node parameters.
  node->declare_parameter<double>("x", desired_position.x());
  node->declare_parameter<double>("y", desired_position.y());
  node->declare_parameter<double>("z", desired_position.z());
  node->declare_parameter<double>("yaw", desired_yaw);
  node->get_parameter("x", desired_position.x());
  node->get_parameter("y", desired_position.y());
  node->get_parameter("z", desired_position.z());
  node->get_parameter("yaw", desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  RCLCPP_INFO(node->get_logger(),
           "Publishing waypoint on namespace %s: [%f, %f, %f].",
           node->get_namespace(), desired_position.x(),
           desired_position.y(), desired_position.z());
  trajectory_pub->publish(trajectory_msg);

  rclcpp::spin_some(node);
  rclcpp::shutdown();

  return 0;
}
