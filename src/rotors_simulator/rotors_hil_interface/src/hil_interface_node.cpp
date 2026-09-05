/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
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

#include <functional>
#include <stdexcept>
#include <utility>

#include "rotors_hil_interface/hil_interface_node.h"

namespace rotors_hil {

HilInterfaceNode::HilInterfaceNode()
    : rclcpp::Node("rotors_hil_interface_node") {
  // Keep the ROS 1 private-parameter names and defaults unchanged.  In ROS 2
  // these are node parameters, so launch files can override them with the
  // same names.
  declare_parameter<bool>("sensor_level_hil", kDefaultSensorLevelHil);
  declare_parameter<double>("hil_frequency", kDefaultHilFrequency);
  declare_parameter<double>("body_to_sensor_roll", kDefaultBodyToSensorsRoll);
  declare_parameter<double>("body_to_sensor_pitch",
                            kDefaultBodyToSensorsPitch);
  declare_parameter<double>("body_to_sensor_yaw", kDefaultBodyToSensorsYaw);
  declare_parameter<std::string>(
      "actuators_pub_topic",
      std::string(mav_msgs::default_topics::COMMAND_ACTUATORS));
  declare_parameter<std::string>("mavlink_pub_topic", kDefaultMavlinkPubTopic);
  declare_parameter<std::string>("hil_controls_sub_topic",
                                 kDefaultHilControlsSubTopic);

  const bool sensor_level_hil = get_parameter("sensor_level_hil").as_bool();
  hil_frequency_ = get_parameter("hil_frequency").as_double();
  const double S_B_roll =
      get_parameter("body_to_sensor_roll").as_double();
  const double S_B_pitch =
      get_parameter("body_to_sensor_pitch").as_double();
  const double S_B_yaw =
      get_parameter("body_to_sensor_yaw").as_double();
  const std::string actuators_pub_topic =
      get_parameter("actuators_pub_topic").as_string();
  const std::string mavlink_pub_topic =
      get_parameter("mavlink_pub_topic").as_string();
  const std::string hil_controls_sub_topic =
      get_parameter("hil_controls_sub_topic").as_string();

  if (hil_frequency_ <= 0.0) {
    throw std::invalid_argument("hil_frequency must be greater than zero");
  }

  // Create the quaternion and rotation matrix to rotate data into NED frame.
  Eigen::AngleAxisd roll_angle(S_B_roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_angle(S_B_pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_angle(S_B_yaw, Eigen::Vector3d::UnitZ());

  const Eigen::Quaterniond q_S_B = roll_angle * pitch_angle * yaw_angle;

  if (sensor_level_hil) {
    hil_interface_ =
        std::make_unique<HilSensorLevelInterface>(this, q_S_B);
  } else {
    hil_interface_ =
        std::make_unique<HilStateLevelInterface>(this, q_S_B);
  }

  const auto queue = rclcpp::QoS(rclcpp::KeepLast(1));
  actuators_pub_ =
      create_publisher<mav_msgs::msg::Actuators>(actuators_pub_topic, queue);
  mavlink_pub_ =
      create_publisher<mavros_msgs::msg::Mavlink>(mavlink_pub_topic,
                                                  rclcpp::QoS(5));
  hil_controls_sub_ = create_subscription<mavros_msgs::msg::HilControls>(
      hil_controls_sub_topic, queue,
      std::bind(&HilInterfaceNode::HilControlsCallback, this,
                std::placeholders::_1));
}

void HilInterfaceNode::MainTask() {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(get_node_base_interface());
  rclcpp::Rate rate(hil_frequency_);

  while (rclcpp::ok()) {
    std::vector<mavros_msgs::msg::Mavlink> hil_msgs =
        hil_interface_->CollectData();

    // Preserve the ROS 1 publication order: CollectData() appends GPS before
    // sensor/state data, while the original loop published from the back.
    while (!hil_msgs.empty()) {
      mavlink_pub_->publish(hil_msgs.back());
      hil_msgs.pop_back();
    }

    executor.spin_some();
    rate.sleep();
  }
}

void HilInterfaceNode::HilControlsCallback(
    const mavros_msgs::msg::HilControls::ConstSharedPtr& hil_controls_msg) {
  mav_msgs::msg::Actuators act_msg;

  const rclcpp::Time current_time = get_clock()->now();

  // Keep the original six-channel order exactly: roll, pitch, yaw, aux1,
  // aux2, throttle.
  act_msg.normalized.push_back(hil_controls_msg->roll_ailerons);
  act_msg.normalized.push_back(hil_controls_msg->pitch_elevator);
  act_msg.normalized.push_back(hil_controls_msg->yaw_rudder);
  act_msg.normalized.push_back(hil_controls_msg->aux1);
  act_msg.normalized.push_back(hil_controls_msg->aux2);
  act_msg.normalized.push_back(hil_controls_msg->throttle);

  act_msg.header.stamp =
      static_cast<builtin_interfaces::msg::Time>(current_time);
  actuators_pub_->publish(act_msg);
}

}  // namespace rotors_hil

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto hil_interface_node =
      std::make_shared<rotors_hil::HilInterfaceNode>();
  hil_interface_node->MainTask();
  rclcpp::shutdown();
  return 0;
}
