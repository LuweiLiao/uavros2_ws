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

#include <stdexcept>

#include "rotors_hil_interface/hil_interface.h"

namespace rotors_hil {

HilSensorLevelInterface::HilSensorLevelInterface(
    rclcpp::Node* node, const Eigen::Quaterniond& q_S_B)
    : HilInterface(node), hil_gps_msg_{}, hil_sensor_msg_{},
      gps_interval_nsec_(0) {
  // Retrieve the necessary parameters.  Names and defaults are unchanged
  // from the ROS 1 implementation.
  node_->declare_parameter<double>("gps_frequency", kDefaultGpsFrequency);
  node_->declare_parameter<std::string>(
      "air_speed_topic",
      std::string(mav_msgs::default_topics::AIR_SPEED));
  node_->declare_parameter<std::string>(
      "gps_topic", std::string(mav_msgs::default_topics::GPS));
  node_->declare_parameter<std::string>(
      "ground_speed_topic",
      std::string(mav_msgs::default_topics::GROUND_SPEED));
  node_->declare_parameter<std::string>(
      "imu_topic", std::string(mav_msgs::default_topics::IMU));
  node_->declare_parameter<std::string>(
      "mag_topic", std::string(mav_msgs::default_topics::MAGNETIC_FIELD));
  node_->declare_parameter<std::string>("pressure_topic",
                                        kDefaultPressureSubTopic);

  const double gps_freq = node_->get_parameter("gps_frequency").as_double();
  const std::string air_speed_sub_topic =
      node_->get_parameter("air_speed_topic").as_string();
  const std::string gps_sub_topic =
      node_->get_parameter("gps_topic").as_string();
  const std::string ground_speed_sub_topic =
      node_->get_parameter("ground_speed_topic").as_string();
  const std::string imu_sub_topic =
      node_->get_parameter("imu_topic").as_string();
  const std::string mag_sub_topic =
      node_->get_parameter("mag_topic").as_string();
  const std::string pressure_sub_topic =
      node_->get_parameter("pressure_topic").as_string();

  if (gps_freq <= 0.0) {
    throw std::invalid_argument("gps_frequency must be greater than zero");
  }

  // Compute the desired interval between published GPS messages.
  gps_interval_nsec_ = static_cast<uint64_t>(kSecToNsec / gps_freq);

  // Compute the rotation matrix to rotate data into NED frame.
  q_S_B_ = q_S_B;
  R_S_B_ = q_S_B_.matrix().cast<float>();

  const auto queue = rclcpp::QoS(rclcpp::KeepLast(1));

  // Initialize the subscribers.
  air_speed_sub_ =
      node_->create_subscription<geometry_msgs::msg::TwistStamped>(
          air_speed_sub_topic, queue,
          [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
            hil_listeners_.AirSpeedCallback(msg, &hil_data_);
          });

  gps_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_sub_topic, queue,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
        hil_listeners_.GpsCallback(msg, &hil_data_);
      });

  ground_speed_sub_ =
      node_->create_subscription<geometry_msgs::msg::TwistStamped>(
          ground_speed_sub_topic, queue,
          [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
            hil_listeners_.GroundSpeedCallback(msg, &hil_data_);
          });

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_sub_topic, queue,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        hil_listeners_.ImuCallback(msg, &hil_data_);
      });

  mag_sub_ = node_->create_subscription<sensor_msgs::msg::MagneticField>(
      mag_sub_topic, queue,
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
        hil_listeners_.MagCallback(msg, &hil_data_);
      });

  pressure_sub_ = node_->create_subscription<sensor_msgs::msg::FluidPressure>(
      pressure_sub_topic, queue,
      [this](sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
        hil_listeners_.PressureCallback(msg, &hil_data_);
      });
}

std::vector<mavros_msgs::msg::Mavlink>
HilSensorLevelInterface::CollectData() {
  std::lock_guard<std::mutex> lock(mtx_);

  const rclcpp::Time current_time = node_->get_clock()->now();
  const uint64_t time_usec = RosTimeToMicroseconds(current_time);
  const uint64_t now_nsec =
      current_time.nanoseconds() > 0
          ? static_cast<uint64_t>(current_time.nanoseconds())
          : 0;

  mavlink_message_t mmsg{};
  std::vector<mavros_msgs::msg::Mavlink> hil_msgs;

  // Rotate gyroscope, accelerometer, and magnetometer data into NED frame.
  const Eigen::Vector3f gyro = R_S_B_ * hil_data_.gyro_rad_per_s;
  const Eigen::Vector3f acc = R_S_B_ * hil_data_.acc_m_per_s2;
  const Eigen::Vector3f mag = R_S_B_ * hil_data_.mag_G;

  // Check if we need to publish a HIL_GPS message.
  if (last_gps_pub_time_nsec_ == 0 ||
      now_nsec < last_gps_pub_time_nsec_ ||
      now_nsec - last_gps_pub_time_nsec_ >= gps_interval_nsec_) {
    last_gps_pub_time_nsec_ = now_nsec;

    // Rotate ground speed data into NED frame.
    const Eigen::Vector3i gps_vel =
        (R_S_B_ * hil_data_.gps_vel_cm_per_s.cast<float>()).cast<int>();

    // Fill in a MAVLINK HIL_GPS message and convert it to MAVROS format.
    hil_gps_msg_.time_usec = time_usec;
    hil_gps_msg_.fix_type = hil_data_.fix_type;
    hil_gps_msg_.lat = hil_data_.lat_1e7deg;
    hil_gps_msg_.lon = hil_data_.lon_1e7deg;
    hil_gps_msg_.alt = hil_data_.alt_mm;
    hil_gps_msg_.eph = hil_data_.eph_cm;
    hil_gps_msg_.epv = hil_data_.epv_cm;
    hil_gps_msg_.vel = hil_data_.vel_1e2m_per_s;
    hil_gps_msg_.vn = gps_vel.x();
    hil_gps_msg_.ve = gps_vel.y();
    hil_gps_msg_.vd = gps_vel.z();
    hil_gps_msg_.cog = hil_data_.cog_1e2deg;
    hil_gps_msg_.satellites_visible = hil_data_.satellites_visible;

    mavlink_msg_hil_gps_encode(1, 0, &mmsg, &hil_gps_msg_);

    mavros_msgs::msg::Mavlink rmsg_hil_gps;
    rmsg_hil_gps.header.stamp =
        static_cast<builtin_interfaces::msg::Time>(current_time);
    mavros_msgs::mavlink::convert(mmsg, rmsg_hil_gps);
    hil_msgs.push_back(rmsg_hil_gps);
  }

  // Fill in a MAVLINK HIL_SENSOR message and convert it to MAVROS format.
  hil_sensor_msg_.time_usec = time_usec;
  hil_sensor_msg_.xacc = acc.x();
  hil_sensor_msg_.yacc = acc.y();
  hil_sensor_msg_.zacc = acc.z();
  hil_sensor_msg_.xgyro = gyro.x();
  hil_sensor_msg_.ygyro = gyro.y();
  hil_sensor_msg_.zgyro = gyro.z();
  hil_sensor_msg_.xmag = mag.x();
  hil_sensor_msg_.ymag = mag.y();
  hil_sensor_msg_.zmag = mag.z();
  hil_sensor_msg_.abs_pressure = hil_data_.pressure_abs_mBar;
  hil_sensor_msg_.diff_pressure = hil_data_.pressure_diff_mBar;
  hil_sensor_msg_.pressure_alt = hil_data_.pressure_alt;
  hil_sensor_msg_.temperature = hil_data_.temperature_degC;
  hil_sensor_msg_.fields_updated = kAllFieldsUpdated;

  mavlink_msg_hil_sensor_encode(1, 0, &mmsg, &hil_sensor_msg_);

  mavros_msgs::msg::Mavlink rmsg_hil_sensor;
  rmsg_hil_sensor.header.stamp =
      static_cast<builtin_interfaces::msg::Time>(current_time);
  mavros_msgs::mavlink::convert(mmsg, rmsg_hil_sensor);
  hil_msgs.push_back(rmsg_hil_sensor);

  return hil_msgs;
}

}  // namespace rotors_hil
