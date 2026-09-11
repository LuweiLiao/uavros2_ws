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

#include "rotors_gazebo_plugins/gazebo_ros_interface_plugin.h"
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/vector3d.pb.h>

namespace {
void Require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
template<class T>
void PublishRos(rclcpp::PublisherBase::SharedPtr publisher, const T &message) {
  std::static_pointer_cast<rclcpp::Publisher<T>>(publisher)->publish(message);
}
void HeaderFromNative(const gz::msgs::Header &source, gz_std_msgs::Header *target) {
  target->mutable_stamp()->set_sec(source.stamp().sec());
  target->mutable_stamp()->set_nsec(source.stamp().nsec());
  target->set_frame_id("");
  for (const auto &entry : source.data())
    if (entry.key() == "frame_id" && entry.value_size())
      target->set_frame_id(entry.value(0));
}
void HeaderToNative(const std_msgs::msg::Header &source, gz::msgs::Header *target) {
  target->mutable_stamp()->set_sec(source.stamp.sec);
  target->mutable_stamp()->set_nsec(source.stamp.nanosec);
  auto frame = target->add_data(); frame->set_key("frame_id");
  frame->add_value(source.frame_id);
}
std::string CanonicalTopic(const std::string &value) {
  std::string result = "/";
  for (const char c : value)
    if (c != '/' || result.back() != '/') result += c;
  if (result.size() > 1 && result.back() == '/') result.pop_back();
  return result;
}
std::string ApprovedQuadMotorRosTopic(const std::string &topic) {
  // User-approved ROS2-only exceptions: quad (2026-09-05) and the original
  // tilt_quadcopter's eight motors / four servos (2026-09-06).
  // tsduav_t4's eight motor outputs use the same approved mapping (2026-09-07).
  // Keep SDF, Gazebo private paths and actuator order unchanged.
  for (int motor = 0; motor < 8; ++motor)
    if (topic == "/prop_speed/" + std::to_string(motor))
      return "/prop_speed/motor_" + std::to_string(motor);
  for (int servo = 0; servo < 4; ++servo)
    if (topic == "/tilt_pos/" + std::to_string(servo))
      return "/tilt_pos/servo_" + std::to_string(servo);
  // Scorpio's approved ROS2-only mapping (2026-09-06). The payload remains
  // the original angular velocity; no Gazebo topic or actuator index changes.
  for (const std::string group : {"coxa_pos", "femur_pos", "tibia_pos"})
    for (int joint = 0; joint < 6; ++joint)
      if (topic == "/" + group + "/" + std::to_string(joint))
        return "/" + group + "/joint_" + std::to_string(joint);
  return topic;
}
}  // namespace
#define gzthrow(message) do { std::ostringstream error; error << message; \
  throw std::runtime_error(error.str()); } while (false)

namespace gazebo {
GazeboRosInterfacePlugin::GazeboRosInterfacePlugin() = default;
GazeboRosInterfacePlugin::~GazeboRosInterfacePlugin() { Shutdown(); }

void GazeboRosInterfacePlugin::Shutdown() {
  // No static callback maps: unloading a world releases its ROS/Gazebo routes.
  gz_node_handle_.reset();
  if (executor_) executor_->cancel();
  if (ros_context_ && ros_context_->is_valid())
    ros_context_->shutdown("GazeboRosInterfacePlugin unloaded");
  if (ros_thread_.joinable()) ros_thread_.join();
  ros_subscribers_.clear(); ros_publishers_.clear();
  publisher_nodes_.clear();
  transform_broadcaster_.reset();
  executor_.reset(); ros_node_handle_.reset(); ros_context_.reset();
  routes_.clear();
}

std::string GazeboRosInterfacePlugin::ResolveGazeboTopic(const std::string &topic) const {
  // Gazebo Classic's ~/ expands to /gazebo/<world>/. Keep that transport
  // namespace for the original registration messages and private routes.
  if (topic.rfind("~/", 0) == 0)
    return CanonicalTopic("gazebo/" + world_name_ + "/" + topic.substr(2));
  return CanonicalTopic(topic);
}

void GazeboRosInterfacePlugin::Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) {
  const auto world = ecm.EntityHasComponentType(entity, gz::sim::components::World::typeId)
      ? entity : gz::sim::worldEntity(entity, ecm);
  const auto name = ecm.Component<gz::sim::components::Name>(world);
  if (!name) { gzerr << "[GazeboRosInterfacePlugin] cannot resolve world\n"; return; }
  world_name_ = name->Data();
  try {
    ros_context_ = std::make_shared<rclcpp::Context>();
    ros_context_->init(0, nullptr);
    rclcpp::NodeOptions options; options.context(ros_context_);
    ros_node_handle_ = std::make_shared<rclcpp::Node>(
        "rotors_gazebo_ros_interface_plugin_" + std::to_string(entity), options);
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_handle_);
    rclcpp::ExecutorOptions executorOptions; executorOptions.context = ros_context_;
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
    executor_->add_node(ros_node_handle_);
    ros_thread_ = std::thread([this] { executor_->spin(); });
    gz_node_handle_ = std::make_unique<gz::transport::Node>();
    std::function<void(const gz_std_msgs::ConnectGazeboToRosTopic &)> gzRequest = [this](const auto &msg) {
      try { Require(msg.IsInitialized(), "incomplete bridge request");
        GzConnectGazeboToRosTopicMsgCallback(std::make_shared<const gz_std_msgs::ConnectGazeboToRosTopic>(msg));
      } catch (const std::exception &e) { gzerr << "[GazeboRosInterfacePlugin] " << e.what() << "\n"; }
    };
    std::function<void(const gz_std_msgs::ConnectRosToGazeboTopic &)> rosRequest = [this](const auto &msg) {
      try { Require(msg.IsInitialized(), "incomplete bridge request");
        GzConnectRosToGazeboTopicMsgCallback(std::make_shared<const gz_std_msgs::ConnectRosToGazeboTopic>(msg));
      } catch (const std::exception &e) { gzerr << "[GazeboRosInterfacePlugin] " << e.what() << "\n"; }
    };
    std::function<void(const gz_geometry_msgs::TransformStampedWithFrameIds &)> tf = [this](const auto &msg) {
      try { Require(msg.IsInitialized(), "incomplete transform");
        GzBroadcastTransformMsgCallback(std::make_shared<const gz_geometry_msgs::TransformStampedWithFrameIds>(msg));
      } catch (const std::exception &e) { gzerr << "[GazeboRosInterfacePlugin] " << e.what() << "\n"; }
    };
    Require(gz_node_handle_->Subscribe(ResolveGazeboTopic("~/connect_gazebo_to_ros_subtopic"), gzRequest), "GZ registration subscription failed");
    Require(gz_node_handle_->Subscribe(ResolveGazeboTopic("~/connect_ros_to_gazebo_subtopic"), rosRequest), "ROS registration subscription failed");
    Require(gz_node_handle_->Subscribe(ResolveGazeboTopic("~/broadcast_transform"), tf), "TF subscription failed");
  } catch (...) { Shutdown(); throw; }
}

template<class GazeboMsgT, class RosMsgT>
void GazeboRosInterfacePlugin::ConnectHelper(
    void (GazeboRosInterfacePlugin::*callback)(const std::shared_ptr<const GazeboMsgT> &,
                                             rclcpp::PublisherBase::SharedPtr),
    GazeboRosInterfacePlugin *, std::string, std::string gazeboTopicName,
    std::string rosTopicName, gz::transport::Node *node) {
  auto publisher = ros_node_handle_->create_publisher<RosMsgT>(rosTopicName, rclcpp::QoS(1));
  const auto topic = ResolveGazeboTopic(gazeboTopicName);
  auto forward = [this, callback, publisher](const GazeboMsgT &msg) {
    try {
      Require(msg.IsInitialized(), "incomplete Gazebo message");
      (this->*callback)(std::make_shared<const GazeboMsgT>(msg), publisher);
    } catch (const std::exception &e) { gzerr << "[GazeboRosInterfacePlugin] dropping message: " << e.what() << "\n"; }
  };
  std::function<void(const GazeboMsgT &)> receive = forward;
  Require(node->Subscribe(topic, receive), "Gazebo data subscription failed");
  // The existing motor port uses native Gazebo Sim wire types. Accept those
  // at the same route while retaining the original schemas for other plugins.
  if constexpr (std::is_same_v<GazeboMsgT, gz_std_msgs::Float32>) {
    std::function<void(const gz::msgs::Float &)> native = [forward](const auto &msg) {
      gz_std_msgs::Float32 legacy; legacy.set_data(msg.data()); forward(legacy);
    };
    Require(node->Subscribe(topic, native), "native Float subscription failed");
  } else if constexpr (std::is_same_v<GazeboMsgT, gz_sensor_msgs::Actuators>) {
    std::function<void(const gz::msgs::Actuators &)> native = [forward](const auto &msg) {
      gz_sensor_msgs::Actuators legacy; HeaderFromNative(msg.header(), legacy.mutable_header());
      for (double v : msg.velocity()) legacy.add_angular_velocities(v);
      for (double v : msg.position()) legacy.add_angles(v);
      for (double v : msg.normalized()) legacy.add_normalized(v);
      forward(legacy);
    };
    Require(node->Subscribe(topic, native), "native Actuators subscription failed");
  } else if constexpr (std::is_same_v<GazeboMsgT, gz_mav_msgs::WindSpeed>) {
    std::function<void(const gz::msgs::Vector3d &)> native = [forward](const auto &msg) {
      gz_mav_msgs::WindSpeed legacy; HeaderFromNative(msg.header(), legacy.mutable_header());
      legacy.mutable_velocity()->CopyFrom(msg); forward(legacy);
    };
    Require(node->Subscribe(topic, native), "native wind subscription failed");
  }
  ros_publishers_.push_back(publisher);
}

void GazeboRosInterfacePlugin::GzConnectGazeboToRosTopicMsgCallback(
    GzConnectGazeboToRosTopicMsgPtr &gz_connect_gazebo_to_ros_topic_msg) {
  const std::string gazeboNamespace;
  const auto gazeboTopicName = gz_connect_gazebo_to_ros_topic_msg->gazebo_topic();
  const auto originalRosTopic = gz_connect_gazebo_to_ros_topic_msg->ros_topic();
  const auto rosTopicName = gz_connect_gazebo_to_ros_topic_msg->msgtype() ==
      gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32
      ? ApprovedQuadMotorRosTopic(originalRosTopic) : originalRosTopic;
  const auto key = "gz:" + ResolveGazeboTopic(gazeboTopicName) + ":" + rosTopicName +
      ":" + std::to_string(gz_connect_gazebo_to_ros_topic_msg->msgtype());
  std::lock_guard<std::mutex> lock(registration_mutex_);
  if (!routes_.insert(key).second) return;
  try {
  switch (gz_connect_gazebo_to_ros_topic_msg->msgtype()) {
    case gz_std_msgs::ConnectGazeboToRosTopic::ACTUATORS:
      ConnectHelper<gz_sensor_msgs::Actuators, mav_msgs::msg::Actuators>(
          &GazeboRosInterfacePlugin::GzActuatorsMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::FLOAT_32:
      ConnectHelper<gz_std_msgs::Float32, std_msgs::msg::Float32>(
          &GazeboRosInterfacePlugin::GzFloat32MsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::FLUID_PRESSURE:
      ConnectHelper<gz_sensor_msgs::FluidPressure, sensor_msgs::msg::FluidPressure>(
          &GazeboRosInterfacePlugin::GzFluidPressureMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::IMU:
      ConnectHelper<gz_sensor_msgs::Imu, sensor_msgs::msg::Imu>(
          &GazeboRosInterfacePlugin::GzImuMsgCallback, this, gazeboNamespace,
          gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::JOINT_STATE:
      ConnectHelper<gz_sensor_msgs::JointState, sensor_msgs::msg::JointState>(
          &GazeboRosInterfacePlugin::GzJointStateMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::MAGNETIC_FIELD:
      ConnectHelper<gz_sensor_msgs::MagneticField, sensor_msgs::msg::MagneticField>(
          &GazeboRosInterfacePlugin::GzMagneticFieldMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::NAV_SAT_FIX:
      ConnectHelper<gz_sensor_msgs::NavSatFix, sensor_msgs::msg::NavSatFix>(
          &GazeboRosInterfacePlugin::GzNavSatFixCallback, this, gazeboNamespace,
          gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::POSE:
      ConnectHelper<gz::msgs::Pose, geometry_msgs::msg::Pose>(
          &GazeboRosInterfacePlugin::GzPoseMsgCallback, this, gazeboNamespace,
          gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::POSE_WITH_COVARIANCE_STAMPED:
      ConnectHelper<gz_geometry_msgs::PoseWithCovarianceStamped,
                    geometry_msgs::msg::PoseWithCovarianceStamped>(
          &GazeboRosInterfacePlugin::GzPoseWithCovarianceStampedMsgCallback,
          this, gazeboNamespace, gazeboTopicName, rosTopicName,
          gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::ODOMETRY:
      ConnectHelper<gz_geometry_msgs::Odometry, nav_msgs::msg::Odometry>(
          &GazeboRosInterfacePlugin::GzOdometryMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::TRANSFORM_STAMPED:
      ConnectHelper<gz_geometry_msgs::TransformStamped,
                    geometry_msgs::msg::TransformStamped>(
          &GazeboRosInterfacePlugin::GzTransformStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::TWIST_STAMPED:
      ConnectHelper<gz_geometry_msgs::TwistStamped,
                    geometry_msgs::msg::TwistStamped>(
          &GazeboRosInterfacePlugin::GzTwistStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::VECTOR_3D_STAMPED:
      ConnectHelper<gz_geometry_msgs::Vector3dStamped,
                    geometry_msgs::msg::PointStamped>(
          &GazeboRosInterfacePlugin::GzVector3dStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::WIND_SPEED:
      ConnectHelper<gz_mav_msgs::WindSpeed,
                    rotors_comm::msg::WindSpeed>(
          &GazeboRosInterfacePlugin::GzWindSpeedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    case gz_std_msgs::ConnectGazeboToRosTopic::WRENCH_STAMPED:
      ConnectHelper<gz_geometry_msgs::WrenchStamped,
                    geometry_msgs::msg::WrenchStamped>(
          &GazeboRosInterfacePlugin::GzWrenchStampedMsgCallback, this,
          gazeboNamespace, gazeboTopicName, rosTopicName, gz_node_handle_.get());
      break;
    default:
      gzthrow("ConnectGazeboToRosTopic message type with enum val = "
              << gz_connect_gazebo_to_ros_topic_msg->msgtype()
              << " is not supported by GazeboRosInterfacePlugin.");
  }

    if (rosTopicName != originalRosTopic)
      gzmsg << "[GazeboRosInterfacePlugin] approved ROS2 output mapping "
            << originalRosTopic << " -> " << rosTopicName << "\n";
  } catch (...) { routes_.erase(key); throw; }
}

template<class GazeboMsgT, class RosMsgT>
void GazeboRosInterfacePlugin::ConnectRosHelper(
    void (GazeboRosInterfacePlugin::*callback)(const typename RosMsgT::ConstSharedPtr &,
                                             gz::transport::Node::Publisher),
    const std::string &gazeboTopic, const std::string &rosTopic) {
  const auto topic = ResolveGazeboTopic(gazeboTopic);
  // Gazebo transport requires separate Node instances to advertise distinct
  // wire schemas on one topic. Keep both adapters owned by this world bridge.
  auto legacyNode = std::make_unique<gz::transport::Node>();
  auto nativeNode = std::make_unique<gz::transport::Node>();
  auto publisher = legacyNode->Advertise<GazeboMsgT>(topic);
  Require(static_cast<bool>(publisher), "Gazebo publisher failed");
  gz::transport::Node::Publisher native;
  if constexpr (std::is_same_v<RosMsgT, mav_msgs::msg::Actuators>)
    native = nativeNode->Advertise<gz::msgs::Actuators>(topic);
  else if constexpr (std::is_same_v<RosMsgT, rotors_comm::msg::WindSpeed>)
    native = nativeNode->Advertise<gz::msgs::Vector3d>(topic);
  if constexpr (std::is_same_v<RosMsgT, mav_msgs::msg::Actuators> ||
                std::is_same_v<RosMsgT, rotors_comm::msg::WindSpeed>)
    Require(static_cast<bool>(native), "native Gazebo publisher failed");
  rclcpp::SubscriptionOptions options;
  // Avoid reflecting this bridge's own ROS publication back into its source
  // when a native producer/consumer pair shares a canonical transport topic.
  options.ignore_local_publications = true;
  std::function<void(typename RosMsgT::ConstSharedPtr)> receive =
      [this, callback, publisher, native](typename RosMsgT::ConstSharedPtr msg) mutable {
        (this->*callback)(msg, publisher);
        if constexpr (std::is_same_v<RosMsgT, mav_msgs::msg::Actuators>) {
          gz::msgs::Actuators message;
          for (double v : msg->angular_velocities) message.add_velocity(v);
          if constexpr (std::is_same_v<GazeboMsgT, gz_sensor_msgs::Actuators>) {
            HeaderToNative(msg->header, message.mutable_header());
            for (double v : msg->angles) message.add_position(v);
            for (double v : msg->normalized) message.add_normalized(v);
          }
          native.Publish(message);
        } else if constexpr (std::is_same_v<RosMsgT, rotors_comm::msg::WindSpeed>) {
          gz::msgs::Vector3d message; HeaderToNative(msg->header, message.mutable_header());
          message.set_x(msg->velocity.x); message.set_y(msg->velocity.y); message.set_z(msg->velocity.z);
          native.Publish(message);
        }
      };
  auto subscriber = ros_node_handle_->create_subscription<RosMsgT>(
      rosTopic, rclcpp::QoS(1), receive, options);
  ros_subscribers_.push_back(subscriber);
  publisher_nodes_.push_back(std::move(legacyNode));
  publisher_nodes_.push_back(std::move(nativeNode));
}

void GazeboRosInterfacePlugin::GzConnectRosToGazeboTopicMsgCallback(
    GzConnectRosToGazeboTopicMsgPtr &request) {
  const auto key = "ros:" + ResolveGazeboTopic(request->gazebo_topic()) + ":" +
      request->ros_topic() + ":" + std::to_string(request->msgtype());
  std::lock_guard<std::mutex> lock(registration_mutex_);
  if (!routes_.insert(key).second) return;
  try {
    switch (request->msgtype()) {
      case gz_std_msgs::ConnectRosToGazeboTopic::ACTUATORS:
        ConnectRosHelper<gz_sensor_msgs::Actuators, mav_msgs::msg::Actuators>(
            &GazeboRosInterfacePlugin::RosActuatorsMsgCallback, request->gazebo_topic(), request->ros_topic()); break;
      case gz_std_msgs::ConnectRosToGazeboTopic::COMMAND_MOTOR_SPEED:
        ConnectRosHelper<gz_mav_msgs::CommandMotorSpeed, mav_msgs::msg::Actuators>(
            &GazeboRosInterfacePlugin::RosCommandMotorSpeedMsgCallback, request->gazebo_topic(), request->ros_topic()); break;
      case gz_std_msgs::ConnectRosToGazeboTopic::ROLL_PITCH_YAWRATE_THRUST:
        ConnectRosHelper<gz_mav_msgs::RollPitchYawrateThrust, mav_msgs::msg::RollPitchYawrateThrust>(
            &GazeboRosInterfacePlugin::RosRollPitchYawrateThrustMsgCallback, request->gazebo_topic(), request->ros_topic()); break;
      case gz_std_msgs::ConnectRosToGazeboTopic::WIND_SPEED:
        ConnectRosHelper<gz_mav_msgs::WindSpeed, rotors_comm::msg::WindSpeed>(
            &GazeboRosInterfacePlugin::RosWindSpeedMsgCallback, request->gazebo_topic(), request->ros_topic()); break;
      default: throw std::runtime_error("unsupported ROS-to-Gazebo message enum");
    }
  } catch (...) { routes_.erase(key); throw; }
}

// Original field-by-field converters, mechanically adapted to ROS2 types.
// Each callback owns its message so parallel Gazebo callbacks cannot race.
void GazeboRosInterfacePlugin::ConvertHeaderGzToRos(
    const gz_std_msgs::Header& gz_header,
    std_msgs::msg::Header* ros_header) {
  ros_header->stamp.sec = gz_header.stamp().sec();
  ros_header->stamp.nanosec = gz_header.stamp().nsec();
  ros_header->frame_id = gz_header.frame_id();
}

void GazeboRosInterfacePlugin::ConvertHeaderRosToGz(
    const std_msgs::msg::Header& ros_header,
    gz_std_msgs::Header* gz_header) {
  gz_header->mutable_stamp()->set_sec(ros_header.stamp.sec);
  gz_header->mutable_stamp()->set_nsec(ros_header.stamp.nanosec);
  gz_header->set_frame_id(ros_header.frame_id);
}

//===========================================================================//
//================ GAZEBO -> ROS MSG CALLBACKS/CONVERTERS ===================//
//===========================================================================//

void GazeboRosInterfacePlugin::GzActuatorsMsgCallback(
    GzActuatorsMsgPtr& gz_actuators_msg, rclcpp::PublisherBase::SharedPtr ros_publisher) {
  mav_msgs::msg::Actuators ros_actuators_msg_;
  // We need to convert the Acutuators message from a Gazebo message to a
  // ROS message and then publish it to the ROS framework

  ConvertHeaderGzToRos(gz_actuators_msg->header(), &ros_actuators_msg_.header);

  ros_actuators_msg_.angular_velocities.resize(
      gz_actuators_msg->angular_velocities_size());
  for (int i = 0; i < gz_actuators_msg->angular_velocities_size(); i++) {
    ros_actuators_msg_.angular_velocities[i] =
        gz_actuators_msg->angular_velocities(i);
  }

  // Publish to ROS.
  PublishRos(ros_publisher, ros_actuators_msg_);
}

void GazeboRosInterfacePlugin::GzFloat32MsgCallback(
    GzFloat32MsgPtr& gz_float_32_msg, rclcpp::PublisherBase::SharedPtr ros_publisher) {
  std_msgs::msg::Float32 ros_float_32_msg_;
  // Convert Gazebo message to ROS message
  ros_float_32_msg_.data = gz_float_32_msg->data();

  // Publish to ROS
  PublishRos(ros_publisher, ros_float_32_msg_);
}

void GazeboRosInterfacePlugin::GzFluidPressureMsgCallback(
    GzFluidPressureMsgPtr &gz_fluid_pressure_msg,
    rclcpp::PublisherBase::SharedPtr ros_publisher) {
  sensor_msgs::msg::FluidPressure ros_fluid_pressure_msg_;
  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the FluidPressure message onto ROS.

  ConvertHeaderGzToRos(gz_fluid_pressure_msg->header(),
                       &ros_fluid_pressure_msg_.header);

  ros_fluid_pressure_msg_.fluid_pressure =
      gz_fluid_pressure_msg->fluid_pressure();

  ros_fluid_pressure_msg_.variance = gz_fluid_pressure_msg->variance();

  // Publish to ROS.
  PublishRos(ros_publisher, ros_fluid_pressure_msg_);
}

void GazeboRosInterfacePlugin::GzImuMsgCallback(GzImuPtr& gz_imu_msg,
                                                rclcpp::PublisherBase::SharedPtr ros_publisher) {
  sensor_msgs::msg::Imu ros_imu_msg_;
  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the IMU message onto ROS

  ConvertHeaderGzToRos(gz_imu_msg->header(), &ros_imu_msg_.header);

  ros_imu_msg_.orientation.x = gz_imu_msg->orientation().x();
  ros_imu_msg_.orientation.y = gz_imu_msg->orientation().y();
  ros_imu_msg_.orientation.z = gz_imu_msg->orientation().z();
  ros_imu_msg_.orientation.w = gz_imu_msg->orientation().w();

  // Orientation covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  Require(gz_imu_msg->orientation_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 orientation covariance "
            "elements.");
  Require(
      ros_imu_msg_.orientation_covariance.size() == 9,
      "The ROS IMU message does not have 9 orientation covariance elements.");
  for (int i = 0; i < gz_imu_msg->orientation_covariance_size(); i++) {
    ros_imu_msg_.orientation_covariance[i] =
        gz_imu_msg->orientation_covariance(i);
  }

  ros_imu_msg_.angular_velocity.x = gz_imu_msg->angular_velocity().x();
  ros_imu_msg_.angular_velocity.y = gz_imu_msg->angular_velocity().y();
  ros_imu_msg_.angular_velocity.z = gz_imu_msg->angular_velocity().z();

  Require(gz_imu_msg->angular_velocity_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 angular velocity "
            "covariance elements.");
  Require(ros_imu_msg_.angular_velocity_covariance.size() == 9,
            "The ROS IMU message does not have 9 angular velocity covariance "
            "elements.");
  for (int i = 0; i < gz_imu_msg->angular_velocity_covariance_size(); i++) {
    ros_imu_msg_.angular_velocity_covariance[i] =
        gz_imu_msg->angular_velocity_covariance(i);
  }

  ros_imu_msg_.linear_acceleration.x = gz_imu_msg->linear_acceleration().x();
  ros_imu_msg_.linear_acceleration.y = gz_imu_msg->linear_acceleration().y();
  ros_imu_msg_.linear_acceleration.z = gz_imu_msg->linear_acceleration().z();

  Require(gz_imu_msg->linear_acceleration_covariance_size() == 9,
            "The Gazebo IMU message does not have 9 linear acceleration "
            "covariance elements.");
  Require(ros_imu_msg_.linear_acceleration_covariance.size() == 9,
            "The ROS IMU message does not have 9 linear acceleration "
            "covariance elements.");
  for (int i = 0; i < gz_imu_msg->linear_acceleration_covariance_size(); i++) {
    ros_imu_msg_.linear_acceleration_covariance[i] =
        gz_imu_msg->linear_acceleration_covariance(i);
  }

  // Publish to ROS.
  PublishRos(ros_publisher, ros_imu_msg_);
}

void GazeboRosInterfacePlugin::GzJointStateMsgCallback(
    GzJointStateMsgPtr& gz_joint_state_msg, rclcpp::PublisherBase::SharedPtr ros_publisher) {
  sensor_msgs::msg::JointState ros_joint_state_msg_;
  ConvertHeaderGzToRos(gz_joint_state_msg->header(),
                       &ros_joint_state_msg_.header);

  ros_joint_state_msg_.name.resize(gz_joint_state_msg->name_size());
  for (int i = 0; i < gz_joint_state_msg->name_size(); i++) {
    ros_joint_state_msg_.name[i] = gz_joint_state_msg->name(i);
  }

  ros_joint_state_msg_.position.resize(gz_joint_state_msg->position_size());
  for (int i = 0; i < gz_joint_state_msg->position_size(); i++) {
    ros_joint_state_msg_.position[i] = gz_joint_state_msg->position(i);
  }

  // Publish to ROS.
  PublishRos(ros_publisher, ros_joint_state_msg_);
}

void GazeboRosInterfacePlugin::GzMagneticFieldMsgCallback(
    GzMagneticFieldMsgPtr& gz_magnetic_field_msg,
    rclcpp::PublisherBase::SharedPtr ros_publisher) {
  sensor_msgs::msg::MagneticField ros_magnetic_field_msg_;
  // We need to convert from a Gazebo message to a ROS message,
  // and then forward the MagneticField message onto ROS

  ConvertHeaderGzToRos(gz_magnetic_field_msg->header(),
                       &ros_magnetic_field_msg_.header);

  ros_magnetic_field_msg_.magnetic_field.x =
      gz_magnetic_field_msg->magnetic_field().x();
  ros_magnetic_field_msg_.magnetic_field.y =
      gz_magnetic_field_msg->magnetic_field().y();
  ros_magnetic_field_msg_.magnetic_field.z =
      gz_magnetic_field_msg->magnetic_field().z();

  // Position covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  Require(gz_magnetic_field_msg->magnetic_field_covariance_size() == 9,
            "The Gazebo MagneticField message does not have 9 magnetic field "
            "covariance elements.");
  Require(ros_magnetic_field_msg_.magnetic_field_covariance.size() == 9,
            "The ROS MagneticField message does not have 9 magnetic field "
            "covariance elements.");
  for (int i = 0; i < gz_magnetic_field_msg->magnetic_field_covariance_size();
       i++) {
    ros_magnetic_field_msg_.magnetic_field_covariance[i] =
        gz_magnetic_field_msg->magnetic_field_covariance(i);
  }

  // Publish to ROS.
  PublishRos(ros_publisher, ros_magnetic_field_msg_);
}

void GazeboRosInterfacePlugin::GzNavSatFixCallback(
    GzNavSatFixPtr& gz_nav_sat_fix_msg, rclcpp::PublisherBase::SharedPtr ros_publisher) {
  sensor_msgs::msg::NavSatFix ros_nav_sat_fix_msg_;
  // We need to convert from a Gazebo message to a ROS message, and then forward
  // the NavSatFix message to ROS.

  ConvertHeaderGzToRos(gz_nav_sat_fix_msg->header(),
                       &ros_nav_sat_fix_msg_.header);

  switch (gz_nav_sat_fix_msg->service()) {
    case gz_sensor_msgs::NavSatFix::SERVICE_GPS:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
      break;
    case gz_sensor_msgs::NavSatFix::SERVICE_GLONASS:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
      break;
    case gz_sensor_msgs::NavSatFix::SERVICE_COMPASS:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
      break;
    case gz_sensor_msgs::NavSatFix::SERVICE_GALILEO:
      ros_nav_sat_fix_msg_.status.service =
          sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
      break;
    default:
      gzthrow(
          "Specific value of enum type gz_sensor_msgs::NavSatFix::Service is "
          "not yet supported.");
  }

  switch (gz_nav_sat_fix_msg->status()) {
    case gz_sensor_msgs::NavSatFix::STATUS_NO_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
      break;
    case gz_sensor_msgs::NavSatFix::STATUS_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      break;
    case gz_sensor_msgs::NavSatFix::STATUS_SBAS_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      break;
    case gz_sensor_msgs::NavSatFix::STATUS_GBAS_FIX:
      ros_nav_sat_fix_msg_.status.status =
          sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      break;
    default:
      gzthrow(
          "Specific value of enum type gz_sensor_msgs::NavSatFix::Status is "
          "not yet supported.");
  }

  ros_nav_sat_fix_msg_.latitude = gz_nav_sat_fix_msg->latitude();
  ros_nav_sat_fix_msg_.longitude = gz_nav_sat_fix_msg->longitude();
  ros_nav_sat_fix_msg_.altitude = gz_nav_sat_fix_msg->altitude();

  switch (gz_nav_sat_fix_msg->position_covariance_type()) {
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      break;
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
      break;
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      break;
    case gz_sensor_msgs::NavSatFix::COVARIANCE_TYPE_KNOWN:
      ros_nav_sat_fix_msg_.position_covariance_type =
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
      break;
    default:
      gzthrow(
          "Specific value of enum type "
          "gz_sensor_msgs::NavSatFix::PositionCovarianceType is not yet "
          "supported.");
  }

  // Position covariance should have 9 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  Require(gz_nav_sat_fix_msg->position_covariance_size() == 9,
            "The Gazebo NavSatFix message does not have 9 position covariance "
            "elements.");
  Require(ros_nav_sat_fix_msg_.position_covariance.size() == 9,
            "The ROS NavSatFix message does not have 9 position covariance "
            "elements.");
  for (int i = 0; i < gz_nav_sat_fix_msg->position_covariance_size(); i++) {
    ros_nav_sat_fix_msg_.position_covariance[i] =
        gz_nav_sat_fix_msg->position_covariance(i);
  }

  // Publish to ROS.
  PublishRos(ros_publisher, ros_nav_sat_fix_msg_);
}

void GazeboRosInterfacePlugin::GzOdometryMsgCallback(
    GzOdometryMsgPtr& gz_odometry_msg, rclcpp::PublisherBase::SharedPtr ros_publisher) {
  nav_msgs::msg::Odometry ros_odometry_msg_;
  Require(gz_odometry_msg->pose().covariance_size() <= 36 &&
          gz_odometry_msg->twist().covariance_size() <= 36,
          "Odometry covariance exceeds 36 elements");
  // We need to convert from a Gazebo message to a ROS message, and then forward
  // the Odometry message to ROS.

  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_odometry_msg->header(), &ros_odometry_msg_.header);

  ros_odometry_msg_.child_frame_id = gz_odometry_msg->child_frame_id();

  // ============================================ //
  // ===================== POSE ================= //
  // ============================================ //
  ros_odometry_msg_.pose.pose.position.x =
      gz_odometry_msg->pose().pose().position().x();
  ros_odometry_msg_.pose.pose.position.y =
      gz_odometry_msg->pose().pose().position().y();
  ros_odometry_msg_.pose.pose.position.z =
      gz_odometry_msg->pose().pose().position().z();

  ros_odometry_msg_.pose.pose.orientation.w =
      gz_odometry_msg->pose().pose().orientation().w();
  ros_odometry_msg_.pose.pose.orientation.x =
      gz_odometry_msg->pose().pose().orientation().x();
  ros_odometry_msg_.pose.pose.orientation.y =
      gz_odometry_msg->pose().pose().orientation().y();
  ros_odometry_msg_.pose.pose.orientation.z =
      gz_odometry_msg->pose().pose().orientation().z();

  for (int i = 0; i < gz_odometry_msg->pose().covariance_size(); i++) {
    ros_odometry_msg_.pose.covariance[i] =
        gz_odometry_msg->pose().covariance(i);
  }

  // ============================================ //
  // ===================== TWIST ================ //
  // ============================================ //
  ros_odometry_msg_.twist.twist.linear.x =
      gz_odometry_msg->twist().twist().linear().x();
  ros_odometry_msg_.twist.twist.linear.y =
      gz_odometry_msg->twist().twist().linear().y();
  ros_odometry_msg_.twist.twist.linear.z =
      gz_odometry_msg->twist().twist().linear().z();

  ros_odometry_msg_.twist.twist.angular.x =
      gz_odometry_msg->twist().twist().angular().x();
  ros_odometry_msg_.twist.twist.angular.y =
      gz_odometry_msg->twist().twist().angular().y();
  ros_odometry_msg_.twist.twist.angular.z =
      gz_odometry_msg->twist().twist().angular().z();

  for (int i = 0; i < gz_odometry_msg->twist().covariance_size(); i++) {
    ros_odometry_msg_.twist.covariance[i] =
        gz_odometry_msg->twist().covariance(i);
  }

  // Publish to ROS framework.
  PublishRos(ros_publisher, ros_odometry_msg_);
}

void GazeboRosInterfacePlugin::GzPoseMsgCallback(GzPoseMsgPtr& gz_pose_msg,
                                                 rclcpp::PublisherBase::SharedPtr ros_publisher) {
  geometry_msgs::msg::Pose ros_pose_msg_;
  ros_pose_msg_.position.x = gz_pose_msg->position().x();
  ros_pose_msg_.position.y = gz_pose_msg->position().y();
  ros_pose_msg_.position.z = gz_pose_msg->position().z();

  ros_pose_msg_.orientation.w = gz_pose_msg->orientation().w();
  ros_pose_msg_.orientation.x = gz_pose_msg->orientation().x();
  ros_pose_msg_.orientation.y = gz_pose_msg->orientation().y();
  ros_pose_msg_.orientation.z = gz_pose_msg->orientation().z();

  PublishRos(ros_publisher, ros_pose_msg_);
}

void GazeboRosInterfacePlugin::GzPoseWithCovarianceStampedMsgCallback(
    GzPoseWithCovarianceStampedMsgPtr& gz_pose_with_covariance_stamped_msg,
    rclcpp::PublisherBase::SharedPtr ros_publisher) {
  geometry_msgs::msg::PoseWithCovarianceStamped ros_pose_with_covariance_stamped_msg_;
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_pose_with_covariance_stamped_msg->header(),
                       &ros_pose_with_covariance_stamped_msg_.header);

  // ============================================ //
  // === POSE (both position and orientation) === //
  // ============================================ //
  ros_pose_with_covariance_stamped_msg_.pose.pose.position.x =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .position()
          .x();
  ros_pose_with_covariance_stamped_msg_.pose.pose.position.y =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .position()
          .y();
  ros_pose_with_covariance_stamped_msg_.pose.pose.position.z =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .position()
          .z();

  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.w =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .w();
  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.x =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .x();
  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.y =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .y();
  ros_pose_with_covariance_stamped_msg_.pose.pose.orientation.z =
      gz_pose_with_covariance_stamped_msg->pose_with_covariance()
          .pose()
          .orientation()
          .z();

  // Covariance should have 36 elements, and both the Gazebo and ROS
  // arrays should be the same size!
  Require(gz_pose_with_covariance_stamped_msg->pose_with_covariance()
                    .covariance_size() == 36,
            "The Gazebo PoseWithCovarianceStamped message does not have 9 "
            "position covariance elements.");
  Require(ros_pose_with_covariance_stamped_msg_.pose.covariance.size() == 36,
            "The ROS PoseWithCovarianceStamped message does not have 9 "
            "position covariance elements.");
  for (int i = 0;
       i < gz_pose_with_covariance_stamped_msg->pose_with_covariance()
               .covariance_size();
       i++) {
    ros_pose_with_covariance_stamped_msg_.pose.covariance[i] =
        gz_pose_with_covariance_stamped_msg->pose_with_covariance().covariance(
            i);
  }

  PublishRos(ros_publisher, ros_pose_with_covariance_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzTransformStampedMsgCallback(
    GzTransformStampedMsgPtr& gz_transform_stamped_msg,
    rclcpp::PublisherBase::SharedPtr ros_publisher) {
  geometry_msgs::msg::TransformStamped ros_transform_stamped_msg_;
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_transform_stamped_msg->header(),
                       &ros_transform_stamped_msg_.header);

  // ============================================ //
  // =========== TRANSFORM, TRANSLATION ========= //
  // ============================================ //
  ros_transform_stamped_msg_.transform.translation.x =
      gz_transform_stamped_msg->transform().translation().x();
  ros_transform_stamped_msg_.transform.translation.y =
      gz_transform_stamped_msg->transform().translation().y();
  ros_transform_stamped_msg_.transform.translation.z =
      gz_transform_stamped_msg->transform().translation().z();

  // ============================================ //
  // ============ TRANSFORM, ROTATION =========== //
  // ============================================ //
  ros_transform_stamped_msg_.transform.rotation.w =
      gz_transform_stamped_msg->transform().rotation().w();
  ros_transform_stamped_msg_.transform.rotation.x =
      gz_transform_stamped_msg->transform().rotation().x();
  ros_transform_stamped_msg_.transform.rotation.y =
      gz_transform_stamped_msg->transform().rotation().y();
  ros_transform_stamped_msg_.transform.rotation.z =
      gz_transform_stamped_msg->transform().rotation().z();

  PublishRos(ros_publisher, ros_transform_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzTwistStampedMsgCallback(
    GzTwistStampedMsgPtr& gz_twist_stamped_msg, rclcpp::PublisherBase::SharedPtr ros_publisher) {
  geometry_msgs::msg::TwistStamped ros_twist_stamped_msg_;
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_twist_stamped_msg->header(),
                       &ros_twist_stamped_msg_.header);

  // ============================================ //
  // =================== TWIST ================== //
  // ============================================ //

  ros_twist_stamped_msg_.twist.linear.x =
      gz_twist_stamped_msg->twist().linear().x();
  ros_twist_stamped_msg_.twist.linear.y =
      gz_twist_stamped_msg->twist().linear().y();
  ros_twist_stamped_msg_.twist.linear.z =
      gz_twist_stamped_msg->twist().linear().z();

  ros_twist_stamped_msg_.twist.angular.x =
      gz_twist_stamped_msg->twist().angular().x();
  ros_twist_stamped_msg_.twist.angular.y =
      gz_twist_stamped_msg->twist().angular().y();
  ros_twist_stamped_msg_.twist.angular.z =
      gz_twist_stamped_msg->twist().angular().z();

  PublishRos(ros_publisher, ros_twist_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzVector3dStampedMsgCallback(
    GzVector3dStampedMsgPtr& gz_vector_3d_stamped_msg,
    rclcpp::PublisherBase::SharedPtr ros_publisher) {
  geometry_msgs::msg::PointStamped ros_position_stamped_msg_;
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_vector_3d_stamped_msg->header(),
                       &ros_position_stamped_msg_.header);

  // ============================================ //
  // ================== POSITION ================ //
  // ============================================ //

  ros_position_stamped_msg_.point.x = gz_vector_3d_stamped_msg->position().x();
  ros_position_stamped_msg_.point.y = gz_vector_3d_stamped_msg->position().y();
  ros_position_stamped_msg_.point.z = gz_vector_3d_stamped_msg->position().z();

  PublishRos(ros_publisher, ros_position_stamped_msg_);
}

void GazeboRosInterfacePlugin::GzWindSpeedMsgCallback(
    GzWindSpeedMsgPtr& gz_wind_speed_msg,
    rclcpp::PublisherBase::SharedPtr ros_publisher) {
  rotors_comm::msg::WindSpeed ros_wind_speed_msg_;
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_wind_speed_msg->header(),
                       &ros_wind_speed_msg_.header);

  // ============================================ //
  // ================== VELOCITY ================ //
  // ============================================ //
  ros_wind_speed_msg_.velocity.x =
      gz_wind_speed_msg->velocity().x();
  ros_wind_speed_msg_.velocity.y =
      gz_wind_speed_msg->velocity().y();
  ros_wind_speed_msg_.velocity.z =
      gz_wind_speed_msg->velocity().z();
  PublishRos(ros_publisher, ros_wind_speed_msg_);
}

void GazeboRosInterfacePlugin::GzWrenchStampedMsgCallback(
    GzWrenchStampedMsgPtr& gz_wrench_stamped_msg,
    rclcpp::PublisherBase::SharedPtr ros_publisher) {
  geometry_msgs::msg::WrenchStamped ros_wrench_stamped_msg_;
  // ============================================ //
  // =================== HEADER ================= //
  // ============================================ //
  ConvertHeaderGzToRos(gz_wrench_stamped_msg->header(),
                       &ros_wrench_stamped_msg_.header);

  // ============================================ //
  // =================== FORCE ================== //
  // ============================================ //
  ros_wrench_stamped_msg_.wrench.force.x =
      gz_wrench_stamped_msg->wrench().force().x();
  ros_wrench_stamped_msg_.wrench.force.y =
      gz_wrench_stamped_msg->wrench().force().y();
  ros_wrench_stamped_msg_.wrench.force.z =
      gz_wrench_stamped_msg->wrench().force().z();

  // ============================================ //
  // ==================== TORQUE ================ //
  // ============================================ //
  ros_wrench_stamped_msg_.wrench.torque.x =
      gz_wrench_stamped_msg->wrench().torque().x();
  ros_wrench_stamped_msg_.wrench.torque.y =
      gz_wrench_stamped_msg->wrench().torque().y();
  ros_wrench_stamped_msg_.wrench.torque.z =
      gz_wrench_stamped_msg->wrench().torque().z();

  PublishRos(ros_publisher, ros_wrench_stamped_msg_);
}

//===========================================================================//
//================ ROS -> GAZEBO MSG CALLBACKS/CONVERTERS ===================//
//===========================================================================//

void GazeboRosInterfacePlugin::RosActuatorsMsgCallback(
    const mav_msgs::msg::Actuators::ConstSharedPtr& ros_actuators_msg_ptr,
    gz::transport::Node::Publisher gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_sensor_msgs::Actuators gz_actuators_msg;

  ConvertHeaderRosToGz(ros_actuators_msg_ptr->header,
                       gz_actuators_msg.mutable_header());

  for (int i = 0; i < ros_actuators_msg_ptr->angles.size(); i++) {
    gz_actuators_msg.add_angles(
        ros_actuators_msg_ptr->angles[i]);
  }

  for (int i = 0; i < ros_actuators_msg_ptr->angular_velocities.size(); i++) {
    gz_actuators_msg.add_angular_velocities(
        ros_actuators_msg_ptr->angular_velocities[i]);
  }

  for (int i = 0; i < ros_actuators_msg_ptr->normalized.size(); i++) {
    gz_actuators_msg.add_normalized(
        ros_actuators_msg_ptr->normalized[i]);
  }

  // Publish to Gazebo
  gz_publisher_ptr.Publish(gz_actuators_msg);
}

void GazeboRosInterfacePlugin::RosCommandMotorSpeedMsgCallback(
    const mav_msgs::msg::Actuators::ConstSharedPtr& ros_actuators_msg_ptr,
    gz::transport::Node::Publisher gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_mav_msgs::CommandMotorSpeed gz_command_motor_speed_msg;

  for (int i = 0; i < ros_actuators_msg_ptr->angular_velocities.size(); i++) {
    gz_command_motor_speed_msg.add_motor_speed(
        ros_actuators_msg_ptr->angular_velocities[i]);
  }

  // Publish to Gazebo
  gz_publisher_ptr.Publish(gz_command_motor_speed_msg);
}

void GazeboRosInterfacePlugin::RosRollPitchYawrateThrustMsgCallback(
    const mav_msgs::msg::RollPitchYawrateThrust::ConstSharedPtr&
        ros_roll_pitch_yawrate_thrust_msg_ptr,
    gz::transport::Node::Publisher gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_mav_msgs::RollPitchYawrateThrust gz_roll_pitch_yawrate_thrust_msg;

  ConvertHeaderRosToGz(ros_roll_pitch_yawrate_thrust_msg_ptr->header,
                       gz_roll_pitch_yawrate_thrust_msg.mutable_header());

  gz_roll_pitch_yawrate_thrust_msg.set_roll(
      ros_roll_pitch_yawrate_thrust_msg_ptr->roll);
  gz_roll_pitch_yawrate_thrust_msg.set_pitch(
      ros_roll_pitch_yawrate_thrust_msg_ptr->pitch);
  gz_roll_pitch_yawrate_thrust_msg.set_yaw_rate(
      ros_roll_pitch_yawrate_thrust_msg_ptr->yaw_rate);

  gz_roll_pitch_yawrate_thrust_msg.mutable_thrust()->set_x(
      ros_roll_pitch_yawrate_thrust_msg_ptr->thrust.x);
  gz_roll_pitch_yawrate_thrust_msg.mutable_thrust()->set_y(
      ros_roll_pitch_yawrate_thrust_msg_ptr->thrust.y);
  gz_roll_pitch_yawrate_thrust_msg.mutable_thrust()->set_z(
      ros_roll_pitch_yawrate_thrust_msg_ptr->thrust.z);

  // Publish to Gazebo
  gz_publisher_ptr.Publish(gz_roll_pitch_yawrate_thrust_msg);
}

void GazeboRosInterfacePlugin::RosWindSpeedMsgCallback(
    const rotors_comm::msg::WindSpeed::ConstSharedPtr& ros_wind_speed_msg_ptr,
    gz::transport::Node::Publisher gz_publisher_ptr) {
  // Convert ROS message to Gazebo message

  gz_mav_msgs::WindSpeed gz_wind_speed_msg;

  ConvertHeaderRosToGz(ros_wind_speed_msg_ptr->header,
                       gz_wind_speed_msg.mutable_header());

  gz_wind_speed_msg.mutable_velocity()->set_x(
      ros_wind_speed_msg_ptr->velocity.x);
  gz_wind_speed_msg.mutable_velocity()->set_y(
      ros_wind_speed_msg_ptr->velocity.y);
  gz_wind_speed_msg.mutable_velocity()->set_z(
      ros_wind_speed_msg_ptr->velocity.z);

  // Publish to Gazebo
  gz_publisher_ptr.Publish(gz_wind_speed_msg);
}



void GazeboRosInterfacePlugin::GzBroadcastTransformMsgCallback(
    GzTransformStampedWithFrameIdsMsgPtr &message) {
  geometry_msgs::msg::TransformStamped transform;
  ConvertHeaderGzToRos(message->header(), &transform.header);
  transform.header.frame_id = message->parent_frame_id();
  transform.child_frame_id = message->child_frame_id();
  const auto &source = message->transform();
  transform.transform.translation.x = source.translation().x();
  transform.transform.translation.y = source.translation().y();
  transform.transform.translation.z = source.translation().z();
  transform.transform.rotation.x = source.rotation().x();
  transform.transform.rotation.y = source.rotation().y();
  transform.transform.rotation.z = source.rotation().z();
  transform.transform.rotation.w = source.rotation().w();
  transform_broadcaster_->sendTransform(transform);
}
}  // namespace gazebo
#undef gzthrow
GZ_ADD_PLUGIN(gazebo::GazeboRosInterfacePlugin, gz::sim::System,
              gazebo::GazeboRosInterfacePlugin::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(gazebo::GazeboRosInterfacePlugin, "GazeboRosInterfacePlugin", "ros_interface_plugin")
