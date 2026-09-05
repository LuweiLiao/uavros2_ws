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

#ifndef ROTORS_HIL_INTERFACE_H_
#define ROTORS_HIL_INTERFACE_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <mav_msgs/msg/actuators.hpp>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/msg/mavlink.hpp>
#include <mavros_msgs/mavlink_convert.hpp>

#include <rclcpp/rclcpp.hpp>

#ifndef MAVLINK_H
typedef mavlink::mavlink_message_t mavlink_message_t;
typedef mavlink::mavlink_status_t mavlink_status_t;
#include <mavlink/v2.0/common/mavlink.h>
#endif

#include <rotors_hil_interface/hil_listeners.h>

namespace rotors_hil {
// Constants
static constexpr int kAllFieldsUpdated = 4095;

// Default values
static constexpr double kDefaultGpsFrequency = 5.0;
static const std::string kDefaultPressureSubTopic = "air_pressure";

/// \brief Convert an ROS 2 time value into a single value in microseconds.
/// \param[in] rostime Time, in ROS 2 format, to be converted.
/// \return Time in microseconds.
inline uint64_t RosTimeToMicroseconds(const rclcpp::Time& rostime) {
  return static_cast<uint64_t>(rostime.nanoseconds() / 1000);
}

class HilInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit HilInterface(rclcpp::Node* node) : node_(node) {}

  /// \brief Destructor
  virtual ~HilInterface() = default;

  /// \brief Gather data collected from ROS messages into MAVLINK messages.
  /// \return Vector of MAVLINK messages (in MAVROS format) to be publised.
  virtual std::vector<mavros_msgs::msg::Mavlink> CollectData() = 0;

 protected:
  /// ROS 2 node used to create subscriptions and read the clock.
  rclcpp::Node* node_;

  /// ROS air speed subscriber.
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      air_speed_sub_;

  /// ROS GPS subscriber.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  /// ROS ground speed subscriber.
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      ground_speed_sub_;

  /// ROS IMU subscriber.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  /// ROS magnetometer subscriber.
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;

  /// ROS air pressure subscriber.
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr
      pressure_sub_;

  /// Rotation, in quaternion form, from body into sensor (NED) frame.
  Eigen::Quaterniond q_S_B_;

  /// Rotation, in matrix form, from body into sensor (NED) frame.
  Eigen::Matrix3f R_S_B_;

  /// Object for storing the latest data.
  HilData hil_data_;

  /// Object with callbacks for receiving data.
  HilListeners hil_listeners_;

  /// Mutex lock for thread safety of reading hil data.
  std::mutex mtx_;
};

class HilSensorLevelInterface : public HilInterface {
 public:
  /// \brief Constructor
  /// \param[in] q_S_B Quaternion rotation from body frame to NED frame.
  HilSensorLevelInterface(rclcpp::Node* node, const Eigen::Quaterniond& q_S_B);

  /// \brief Destructor
  virtual ~HilSensorLevelInterface() = default;

  std::vector<mavros_msgs::msg::Mavlink> CollectData() override;

 private:
  /// MAVLINK HIL_GPS message.
  mavlink_hil_gps_t hil_gps_msg_;

  /// MAVLINK HIL_SENSOR message.
  mavlink_hil_sensor_t hil_sensor_msg_;

  /// Interval between outgoing HIL_GPS messages.
  uint64_t gps_interval_nsec_;

  /// Nanosecond portion of the last HIL_GPS message timestamp.
  uint64_t last_gps_pub_time_nsec_{0};
};

class HilStateLevelInterface : public HilInterface {
 public:
  /// \brief Constructor
  /// \param[in] q_S_B Quaternion rotation from body frame to NED frame.
  HilStateLevelInterface(rclcpp::Node* node, const Eigen::Quaterniond& q_S_B);

  /// \brief Destructor
  virtual ~HilStateLevelInterface() = default;

  std::vector<mavros_msgs::msg::Mavlink> CollectData() override;

 private:
  /// MAVLINK HIL_STATE_QUATERNION message.
  mavlink_hil_state_quaternion_t hil_state_qtrn_msg_;
};
}

#endif // ROTORS_HIL_INTERFACE_H_
