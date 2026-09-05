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
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lee_position_controller_node.h"

#include <algorithm>
#include <chrono>
#include <functional>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerNode::LeePositionControllerNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("lee_position_controller_node", options) {
  InitializeParams();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  cmd_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      mav_msgs::default_topics::COMMAND_POSE, qos,
      std::bind(&LeePositionControllerNode::CommandPoseCallback, this,
                std::placeholders::_1));

  cmd_multi_dof_joint_trajectory_sub_ =
      create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, qos,
          std::bind(&LeePositionControllerNode::MultiDofJointTrajectoryCallback,
                    this, std::placeholders::_1));

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      mav_msgs::default_topics::ODOMETRY, qos,
      std::bind(&LeePositionControllerNode::OdometryCallback, this,
                std::placeholders::_1));

  motor_velocity_reference_pub_ =
      create_publisher<mav_msgs::msg::Actuators>(
          mav_msgs::default_topics::COMMAND_ACTUATORS, qos);
}

LeePositionControllerNode::~LeePositionControllerNode() = default;

void LeePositionControllerNode::InitializeParams() {
  // ROS 2 parameters are node-local, which is the equivalent of the ROS 1
  // private NodeHandle ("~") used by the original node.
  GetRosParameter(*this, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(*this, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(*this, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(*this, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(*this, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(*this, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(*this, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(*this, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(*this, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(*this, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(*this, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(*this, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(*this, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
}

void LeePositionControllerNode::Publish() {}

void LeePositionControllerNode::CommandPoseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg) {
  if (command_timer_) {
    command_timer_->cancel();
  }
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  lee_position_controller_.SetTrajectoryPoint(eigen_reference);
}

void LeePositionControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::msg::MultiDOFJointTrajectory::ConstSharedPtr& msg) {
  if (command_timer_) {
    command_timer_->cancel();
  }
  commands_.clear();
  command_waiting_times_.clear();

  const std::size_t n_commands = msg->points.size();
  if (n_commands < 1) {
    RCLCPP_WARN(get_logger(),
                "Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (std::size_t i = 1; i < n_commands; ++i) {
    const auto& reference_before = msg->points[i - 1];
    const auto& current_reference = msg->points[i];
    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);
    commands_.push_back(eigen_reference);

    const auto before_ns = mav_msgs::durationToNanoseconds(
        reference_before.time_from_start);
    const auto current_ns = mav_msgs::durationToNanoseconds(
        current_reference.time_from_start);
    command_waiting_times_.emplace_back(
        std::chrono::nanoseconds(std::max<int64_t>(0, current_ns - before_ns)));
  }

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (!commands_.empty() && !command_waiting_times_.empty()) {
    const auto delay = command_waiting_times_.front();
    command_waiting_times_.pop_front();
    ScheduleNextCommand(delay);
  }
}

void LeePositionControllerNode::ScheduleNextCommand(
    std::chrono::nanoseconds delay) {
  if (delay <= std::chrono::nanoseconds::zero()) {
    delay = std::chrono::nanoseconds(1);
  }
  command_timer_ = create_wall_timer(
      delay, std::bind(&LeePositionControllerNode::TimedCommandCallback, this));
}

void LeePositionControllerNode::TimedCommandCallback() {
  if (commands_.empty()) {
    if (command_timer_) {
      command_timer_->cancel();
    }
    RCLCPP_WARN(get_logger(), "Commands empty, this should not happen here");
    return;
  }

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (!command_waiting_times_.empty() && !commands_.empty()) {
    const auto delay = command_waiting_times_.front();
    command_waiting_times_.pop_front();
    ScheduleNextCommand(delay);
  } else if (command_timer_) {
    command_timer_->cancel();
  }
}

void LeePositionControllerNode::OdometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg) {
  RCLCPP_INFO_ONCE(get_logger(),
                   "LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(*odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::msg::Actuators actuator_msg;
  actuator_msg.angular_velocities.reserve(
      static_cast<std::size_t>(ref_rotor_velocities.size()));
  for (int i = 0; i < ref_rotor_velocities.size(); ++i) {
    actuator_msg.angular_velocities.push_back(ref_rotor_velocities[i]);
  }
  actuator_msg.header.stamp = odometry_msg->header.stamp;
  motor_velocity_reference_pub_->publish(actuator_msg);
}

}  // namespace rotors_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rotors_control::LeePositionControllerNode>(
      options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
