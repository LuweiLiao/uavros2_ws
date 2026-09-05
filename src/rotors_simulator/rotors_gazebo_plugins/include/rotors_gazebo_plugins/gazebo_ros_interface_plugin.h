/*
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
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

#ifndef ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_MSG_INTERFACE_PLUGIN_H

#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "Actuators.pb.h"
#include "CommandMotorSpeed.pb.h"
#include "Float32.pb.h"
#include "FluidPressure.pb.h"
#include "Imu.pb.h"
#include "JointState.pb.h"
#include "MagneticField.pb.h"
#include "NavSatFix.pb.h"
#include "Odometry.pb.h"
#include "PoseWithCovarianceStamped.pb.h"
#include "RollPitchYawrateThrust.pb.h"
#include "TransformStamped.pb.h"
#include "TransformStampedWithFrameIds.pb.h"
#include "TwistStamped.pb.h"
#include "Vector3dStamped.pb.h"
#include "WindSpeed.pb.h"
#include "WrenchStamped.pb.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rotors_comm/msg/wind_speed.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>

namespace gazebo {
// typedef's to make life easier
typedef const std::shared_ptr<const gz_std_msgs::ConnectGazeboToRosTopic>
    GzConnectGazeboToRosTopicMsgPtr;
typedef const std::shared_ptr<const gz_std_msgs::ConnectRosToGazeboTopic>
    GzConnectRosToGazeboTopicMsgPtr;
typedef const std::shared_ptr<const gz_std_msgs::Float32> GzFloat32MsgPtr;
typedef const std::shared_ptr<const gz_geometry_msgs::Odometry>
    GzOdometryMsgPtr;
typedef const std::shared_ptr<const gz::msgs::Pose> GzPoseMsgPtr;
typedef const std::shared_ptr<
    const gz_geometry_msgs::PoseWithCovarianceStamped>
    GzPoseWithCovarianceStampedMsgPtr;
typedef const std::shared_ptr<const gz_geometry_msgs::TransformStamped>
    GzTransformStampedMsgPtr;
typedef const std::shared_ptr<
    const gz_geometry_msgs::TransformStampedWithFrameIds>
    GzTransformStampedWithFrameIdsMsgPtr;
typedef const std::shared_ptr<const gz_geometry_msgs::TwistStamped>
    GzTwistStampedMsgPtr;
typedef const std::shared_ptr<const gz_geometry_msgs::Vector3dStamped>
    GzVector3dStampedMsgPtr;
typedef const std::shared_ptr<const gz_geometry_msgs::WrenchStamped>
    GzWrenchStampedMsgPtr;
typedef const std::shared_ptr<const gz_mav_msgs::RollPitchYawrateThrust>
    GzRollPitchYawrateThrustPtr;
typedef const std::shared_ptr<const gz_mav_msgs::WindSpeed> GzWindSpeedMsgPtr;
typedef const std::shared_ptr<const gz_sensor_msgs::Actuators>
    GzActuatorsMsgPtr;
typedef const std::shared_ptr<const gz_sensor_msgs::FluidPressure>
    GzFluidPressureMsgPtr;
typedef const std::shared_ptr<const gz_sensor_msgs::Imu> GzImuPtr;
typedef const std::shared_ptr<const gz_sensor_msgs::JointState>
    GzJointStateMsgPtr;
typedef const std::shared_ptr<const gz_sensor_msgs::MagneticField>
    GzMagneticFieldMsgPtr;
typedef const std::shared_ptr<const gz_sensor_msgs::NavSatFix> GzNavSatFixPtr;


// Original world bridge: all ROS client dependencies stay here, not in the
// sensor, motor or controller plugins. Only the Gazebo/ROS APIs are adapted.
class GazeboRosInterfacePlugin final : public gz::sim::System,
    public gz::sim::ISystemConfigure {
 public:
  GazeboRosInterfacePlugin();
  ~GazeboRosInterfacePlugin() override;
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override;

 private:
  std::string ResolveGazeboTopic(const std::string &) const;
  void Shutdown();
  template<class GazeboMsgT, class RosMsgT>
  void ConnectHelper(void (GazeboRosInterfacePlugin::*)(
                         const std::shared_ptr<const GazeboMsgT> &,
                         rclcpp::PublisherBase::SharedPtr),
                     GazeboRosInterfacePlugin *, std::string,
                     std::string, std::string, gz::transport::Node *);
  template<class GazeboMsgT, class RosMsgT>
  void ConnectRosHelper(void (GazeboRosInterfacePlugin::*)(
                         const typename RosMsgT::ConstSharedPtr &,
                         gz::transport::Node::Publisher),
                        const std::string &, const std::string &);
  void GzConnectGazeboToRosTopicMsgCallback(
      GzConnectGazeboToRosTopicMsgPtr& gz_connect_gazebo_to_ros_topic_msg);
  void GzConnectRosToGazeboTopicMsgCallback(
      GzConnectRosToGazeboTopicMsgPtr& gz_connect_ros_to_gazebo_topic_msg);
  void ConvertHeaderGzToRos(
      const gz_std_msgs::Header& gz_header,
      std_msgs::msg::Header* ros_header);
  void ConvertHeaderRosToGz(
      const std_msgs::msg::Header& ros_header,
      gz_std_msgs::Header* gz_header);
  void GzActuatorsMsgCallback(GzActuatorsMsgPtr& gz_actuators_msg,
                              rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzFloat32MsgCallback(GzFloat32MsgPtr& gz_float_32_msg,
                            rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzFluidPressureMsgCallback(GzFluidPressureMsgPtr& gz_fluid_pressure_msg,
                                  rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzImuMsgCallback(GzImuPtr& gz_imu_msg, rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzJointStateMsgCallback(GzJointStateMsgPtr& gz_joint_state_msg,
                               rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzMagneticFieldMsgCallback(GzMagneticFieldMsgPtr& gz_magnetic_field_msg,
                                  rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzNavSatFixCallback(GzNavSatFixPtr& gz_nav_sat_fix_msg,
                           rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzOdometryMsgCallback(GzOdometryMsgPtr& gz_odometry_msg,
                             rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzPoseMsgCallback(GzPoseMsgPtr& gz_pose_msg,
                         rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzPoseWithCovarianceStampedMsgCallback(
      GzPoseWithCovarianceStampedMsgPtr& gz_pose_with_covariance_stamped_msg,
      rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzVector3dStampedMsgCallback(
      GzVector3dStampedMsgPtr& gz_vector_3d_stamped_msg,
      rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzTransformStampedMsgCallback(
      GzTransformStampedMsgPtr& gz_transform_stamped_msg,
      rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzTwistStampedMsgCallback(GzTwistStampedMsgPtr& gz_twist_stamped_msg,
                                 rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzWindSpeedMsgCallback(GzWindSpeedMsgPtr& gz_wind_speed_msg,
                              rclcpp::PublisherBase::SharedPtr ros_publisher);
  void GzWrenchStampedMsgCallback(GzWrenchStampedMsgPtr& gz_wrench_stamped_msg,
                                  rclcpp::PublisherBase::SharedPtr ros_publisher);
  void RosActuatorsMsgCallback(
      const mav_msgs::msg::Actuators::ConstSharedPtr& ros_actuators_msg_ptr,
      gz::transport::Node::Publisher gz_publisher_ptr);
  void RosCommandMotorSpeedMsgCallback(
      const mav_msgs::msg::Actuators::ConstSharedPtr& ros_command_motor_speed_msg_ptr,
      gz::transport::Node::Publisher gz_publisher_ptr);
  void RosRollPitchYawrateThrustMsgCallback(
      const mav_msgs::msg::RollPitchYawrateThrust::ConstSharedPtr&
          ros_roll_pitch_yawrate_thrust_msg_ptr,
      gz::transport::Node::Publisher gz_publisher_ptr);
  void RosWindSpeedMsgCallback(
      const rotors_comm::msg::WindSpeed::ConstSharedPtr& ros_wind_speed_msg_ptr,
      gz::transport::Node::Publisher gz_publisher_ptr);
  void GzBroadcastTransformMsgCallback(
      GzTransformStampedWithFrameIdsMsgPtr& broadcast_transform_msg);

  std::string world_name_;
  std::mutex registration_mutex_;
  std::set<std::string> routes_;
  std::shared_ptr<rclcpp::Context> ros_context_;
  rclcpp::Node::SharedPtr ros_node_handle_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread ros_thread_;
  std::vector<rclcpp::PublisherBase::SharedPtr> ros_publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> ros_subscribers_;
  std::vector<std::unique_ptr<gz::transport::Node>> publisher_nodes_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  // Shutdown destroys transport subscriptions before any callback state.
  std::unique_ptr<gz::transport::Node> gz_node_handle_;
};
}  // namespace gazebo
#endif
