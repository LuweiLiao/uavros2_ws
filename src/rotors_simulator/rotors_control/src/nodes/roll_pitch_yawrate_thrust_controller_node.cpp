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

#include "roll_pitch_yawrate_thrust_controller_node.h"

#include <functional>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustControllerNode(
    const rclcpp::NodeOptions& options)
    : rclcpp::Node("roll_pitch_yawrate_thrust_controller_node", options) {
  InitializeParams();

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  cmd_roll_pitch_yawrate_thrust_sub_ =
      create_subscription<mav_msgs::msg::RollPitchYawrateThrust>(
          kDefaultCommandRollPitchYawrateThrustTopic, qos,
          std::bind(&RollPitchYawrateThrustControllerNode::
                        RollPitchYawrateThrustCallback,
                    this, std::placeholders::_1));
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      kDefaultOdometryTopic, qos,
      std::bind(&RollPitchYawrateThrustControllerNode::OdometryCallback, this,
                std::placeholders::_1));
  motor_velocity_reference_pub_ =
      create_publisher<mav_msgs::msg::Actuators>(
          kDefaultCommandMotorSpeedTopic, qos);
}

RollPitchYawrateThrustControllerNode::~RollPitchYawrateThrustControllerNode() =
    default;

void RollPitchYawrateThrustControllerNode::InitializeParams() {
  GetRosParameter(*this, "attitude_gain/x",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(*this, "attitude_gain/y",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(*this, "attitude_gain/z",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(*this, "angular_rate_gain/x",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(*this, "angular_rate_gain/y",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(*this, "angular_rate_gain/z",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(*this,
                       &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_);
  roll_pitch_yawrate_thrust_controller_.InitializeParameters();
}

void RollPitchYawrateThrustControllerNode::Publish() {}

void RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustCallback(
    const mav_msgs::msg::RollPitchYawrateThrust::ConstSharedPtr& msg) {
  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust;
  mav_msgs::eigenRollPitchYawrateThrustFromMsg(
      *msg, &roll_pitch_yawrate_thrust);
  roll_pitch_yawrate_thrust_controller_.SetRollPitchYawrateThrust(
      roll_pitch_yawrate_thrust);
}

void RollPitchYawrateThrustControllerNode::OdometryCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr& odometry_msg) {
  RCLCPP_INFO_ONCE(
      get_logger(),
      "RollPitchYawrateThrustController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(*odometry_msg, &odometry);
  roll_pitch_yawrate_thrust_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  roll_pitch_yawrate_thrust_controller_.CalculateRotorVelocities(
      &ref_rotor_velocities);

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
  auto node =
      std::make_shared<rotors_control::RollPitchYawrateThrustControllerNode>(
          options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
