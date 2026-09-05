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


#include "rotors_joy_interface/joy.h"

#include <mav_msgs/default_topics.h>

#include <cmath>

namespace {

template <typename T>
T GetParameter(rclcpp::Node &node, const std::string &name, const T &value)
{
  node.declare_parameter<T>(name, value);
  return node.get_parameter(name).get_value<T>();
}

}  // namespace

Joy::Joy()
  : rclcpp::Node("rotors_joy_interface")
{
  ctrl_pub_ = create_publisher<mav_msgs::msg::RollPitchYawrateThrust>(
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST,
    rclcpp::QoS(rclcpp::KeepLast(10)));

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
  current_yaw_vel_ = 0;

  axes_.roll = GetParameter(*this, "axis_roll_", 0);
  axes_.pitch = GetParameter(*this, "axis_pitch_", 1);
  axes_.thrust = GetParameter(*this, "axis_thrust_", 2);

  axes_.roll_direction = GetParameter(*this, "axis_direction_roll", -1);
  axes_.pitch_direction = GetParameter(*this, "axis_direction_pitch", 1);
  axes_.thrust_direction = GetParameter(*this, "axis_direction_thrust", 1);

  max_.v_xy = GetParameter(*this, "max_v_xy", 1.0);  // [m/s]
  max_.roll = GetParameter(*this, "max_roll", 10.0 * M_PI / 180.0);  // [rad]
  max_.pitch = GetParameter(*this, "max_pitch", 10.0 * M_PI / 180.0);  // [rad]
  max_.rate_yaw = GetParameter(*this, "max_yaw_rate", 45.0 * M_PI / 180.0);  // [rad/s]
  max_.thrust = GetParameter(*this, "max_thrust", 30.0);  // [N]

  v_yaw_step_ = GetParameter(*this, "v_yaw_step", 0.05);  // [rad/s]

  is_fixed_wing_ = GetParameter(*this, "is_fixed_wing", false);

  buttons_.yaw_left = GetParameter(*this, "button_yaw_left_", 3);
  buttons_.yaw_right = GetParameter(*this, "button_yaw_right_", 4);
  buttons_.ctrl_enable = GetParameter(*this, "button_ctrl_enable_", 5);
  buttons_.ctrl_mode = GetParameter(*this, "button_ctrl_mode_", 10);
  buttons_.takeoff = GetParameter(*this, "button_takeoff_", 7);
  buttons_.land = GetParameter(*this, "button_land_", 8);

  namespace_ = get_namespace();
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::SensorDataQoS(),
    std::bind(&Joy::JoyCallback, this, std::placeholders::_1));
}

void Joy::StopMav() {
  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;
}

void Joy::JoyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
{
  const auto valid_axis = [msg](int index) {
    return index >= 0 && static_cast<std::size_t>(index) < msg->axes.size();
  };
  const auto valid_button = [msg](int index) {
    return index >= 0 && static_cast<std::size_t>(index) < msg->buttons.size();
  };

  if (!valid_axis(axes_.roll) || !valid_axis(axes_.pitch) ||
      !valid_axis(axes_.thrust)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Joy message does not contain configured axes");
    return;
  }

  current_joy_ = *msg;
  control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
  control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;

  if (valid_button(buttons_.yaw_left) && msg->buttons[buttons_.yaw_left]) {
    current_yaw_vel_ = max_.rate_yaw;
  }
  else if (valid_button(buttons_.yaw_right) && msg->buttons[buttons_.yaw_right]) {
    current_yaw_vel_ = -max_.rate_yaw;
  }
  else {
    current_yaw_vel_ = 0;
  }
  control_msg_.yaw_rate = current_yaw_vel_;

  if (is_fixed_wing_) {
    double thrust = msg->axes[axes_.thrust] * axes_.thrust_direction;
    control_msg_.thrust.x = (thrust >= 0.0) ? thrust : 0.0;
  }
  else {
    control_msg_.thrust.z = (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction;
  }

  control_msg_.header.stamp = now();
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void Joy::Publish()
{
  ctrl_pub_->publish(control_msg_);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto joy = std::make_shared<Joy>();
  rclcpp::spin(joy);
  rclcpp::shutdown();
  return 0;
}
