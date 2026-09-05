/*
 * Gazebo Sim port of the ROS 1 PoleMavlinkPosePlugin.
 *
 * The original source filename, class name, shared-library name, SDF keys,
 * TCP/MAVLink wire contract and ROS target topic are preserved.  Gazebo Sim
 * systems are used only as the API boundary; the odometry fields and NED
 * frame conversion remain the ROS 1 behavior.
 */

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <mavlink/v2.0/common/mavlink.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{

class PoleMavlinkPosePlugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
public:
  PoleMavlinkPosePlugin() = default;

  ~PoleMavlinkPosePlugin() override
  {
    CloseSocket();
    ShutdownRos();
  }

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    model_entity_ = _entity;
    model_ = gz::sim::Model(model_entity_);

    link_name_ = _sdf->Get("linkName", std::string("pole")).first;
    tcp_addr_ = _sdf->Get("tcpAddr", tcp_addr_).first;
    tcp_port_ = _sdf->Get("tcpPort", tcp_port_).first;
    send_rate_hz_ = _sdf->Get("sendRateHz", send_rate_hz_).first;
    ros_target_topic_ = _sdf->Get(
        "rosTargetTopic", ros_target_topic_).first;
    ros_frame_id_ = _sdf->Get("rosFrameId", ros_frame_id_).first;
    odom_point_offset_link_ = _sdf->Get(
        "odomPointOffset", odom_point_offset_link_).first;
    system_id_ = static_cast<uint8_t>(_sdf->Get(
        "systemId", static_cast<int>(system_id_)).first);
    component_id_ = static_cast<uint8_t>(_sdf->Get(
        "componentId", static_cast<int>(component_id_)).first);

    ResolveLink(_ecm);
    if (link_entity_ == gz::sim::kNullEntity)
      return;

    if (tcp_port_ <= 0 || tcp_port_ > 65535)
    {
      gzerr << "[PoleMavlinkPosePlugin] Missing or invalid <tcpPort> in SDF\n";
      return;
    }
    if (send_rate_hz_ <= 0.0)
    {
      gzerr << "[PoleMavlinkPosePlugin] <sendRateHz> must be positive\n";
      return;
    }

    // Match the ROS 1 startup behavior: make an initial connection attempt,
    // then retry from PreUpdate if SITL is not listening yet.
    OpenSocket();
    InitializeRos();

    gzmsg << "[PoleMavlinkPosePlugin] Sending "
          << gz::sim::scopedName(model_entity_, _ecm) << "::" << link_name_
          << " point offset " << odom_point_offset_link_
          << " as MAVLink ODOMETRY to " << tcp_addr_ << ":" << tcp_port_
          << " via TCP at " << send_rate_hz_ << " Hz\n";
    gzmsg << "[PoleMavlinkPosePlugin] Listening for ROS pole position commands on ["
          << ros_target_topic_ << "] frame [" << ros_frame_id_
          << "]. Commands are ArduPilot local NED; ODOMETRY uses real pole "
             "pose converted to NED.\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!ResolveLink(_ecm) || send_rate_hz_ <= 0.0)
      return;

    if (clear_velocity_commands_)
    {
      _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
          link_entity_);
      _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(
          link_entity_);
      clear_velocity_commands_ = false;
    }

    const double now = std::chrono::duration<double>(_info.simTime).count();
    ApplyPendingPositionCommand(_ecm);

    if (socket_fd_ < 0)
    {
      if (last_connect_attempt_time_ < 0.0 ||
          now - last_connect_attempt_time_ > 1.0)
      {
        last_connect_attempt_time_ = now;
        OpenSocket();
      }
      if (socket_fd_ < 0)
        return;
    }

    if (last_send_time_ >= 0.0 &&
        now - last_send_time_ < 1.0 / send_rate_hz_)
      return;
    last_send_time_ = now;

    const auto pose = gz::sim::Link(link_entity_).WorldPose(_ecm);
    const auto linear_velocity = gz::sim::Link(link_entity_)
        .WorldLinearVelocity(_ecm);
    const auto angular_velocity = gz::sim::Link(link_entity_)
        .WorldAngularVelocity(_ecm);
    if (!pose)
      return;

    const gz::math::Vector3d point_offset_world =
        pose->Rot().RotateVector(odom_point_offset_link_);
    const gz::math::Vector3d position = pose->Pos() + point_offset_world;
    const gz::math::Vector3d linear = linear_velocity.value_or(
        gz::math::Vector3d::Zero);
    const gz::math::Vector3d angular = angular_velocity.value_or(
        gz::math::Vector3d::Zero);
    const gz::math::Vector3d velocity =
        linear + angular.Cross(point_offset_world);

    SendOdometry(static_cast<uint64_t>(now * 1.0e6),
                 WorldToNed(position), WorldToNed(velocity));
  }

  void Reset(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager & /*_ecm*/) override
  {
    last_send_time_ = -1.0;
    last_connect_attempt_time_ = -1.0;
  }

private:
  bool ResolveLink(gz::sim::EntityComponentManager &_ecm)
  {
    if (link_entity_ == gz::sim::kNullEntity)
      link_entity_ = model_.LinkByName(_ecm, link_name_);
    if (link_entity_ != gz::sim::kNullEntity)
    {
      gz::sim::Link(link_entity_).EnableVelocityChecks(_ecm);
      return true;
    }

    if (!warned_missing_link_)
    {
      gzerr << "[PoleMavlinkPosePlugin] Link [" << link_name_
            << "] not found in model entity [" << model_entity_ << "]\n";
      warned_missing_link_ = true;
    }
    return false;
  }

  void InitializeRos()
  {
    try
    {
      ros_context_ = std::make_shared<rclcpp::Context>();
      ros_context_->init(0, nullptr);
      rclcpp::NodeOptions node_options;
      node_options.context(ros_context_);
      ros_node_ = std::make_shared<rclcpp::Node>(
          "pole_mavlink_pose_plugin", node_options);

      target_position_sub_ =
          ros_node_->create_subscription<geometry_msgs::msg::PointStamped>(
              ros_target_topic_, rclcpp::QoS(rclcpp::KeepLast(1)),
              [this](geometry_msgs::msg::PointStamped::ConstSharedPtr _message)
              {
                std::lock_guard<std::mutex> lock(target_mutex_);
                pending_position_ned_ = gz::math::Vector3d(
                    _message->point.x, _message->point.y,
                    _message->point.z);
                has_pending_position_ = true;
                gzmsg << "[PoleMavlinkPosePlugin] Received ROS pole position command NED: "
                      << pending_position_ned_ << "\n";
              });

      rclcpp::ExecutorOptions executor_options;
      executor_options.context = ros_context_;
      executor_ = std::make_unique<
          rclcpp::executors::SingleThreadedExecutor>(executor_options);
      executor_->add_node(ros_node_);
      ros_thread_ = std::thread([this]()
      {
        if (executor_)
          executor_->spin();
      });
    }
    catch (const std::exception &error)
    {
      gzerr << "[PoleMavlinkPosePlugin] ROS 2 initialization failed: "
            << error.what() << "\n";
      ShutdownRos();
    }
  }

  void ShutdownRos()
  {
    if (executor_)
      executor_->cancel();
    if (ros_context_ && ros_context_->is_valid())
      ros_context_->shutdown("PoleMavlinkPosePlugin shutdown");
    if (ros_thread_.joinable())
      ros_thread_.join();
    if (executor_ && ros_node_)
      executor_->remove_node(ros_node_);
    target_position_sub_.reset();
    ros_node_.reset();
    executor_.reset();
    ros_context_.reset();
  }

  bool OpenSocket()
  {
    if (socket_fd_ >= 0)
      return true;

    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0)
    {
      gzerr << "[PoleMavlinkPosePlugin] Failed to create TCP socket\n";
      return false;
    }

    std::memset(&remote_addr_, 0, sizeof(remote_addr_));
    remote_addr_.sin_family = AF_INET;
    remote_addr_.sin_port = htons(static_cast<uint16_t>(tcp_port_));
    if (inet_aton(tcp_addr_.c_str(), &remote_addr_.sin_addr) == 0)
    {
      gzerr << "[PoleMavlinkPosePlugin] Invalid TCP address [" << tcp_addr_
            << "]\n";
      CloseSocket();
      return false;
    }

    if (connect(socket_fd_, reinterpret_cast<sockaddr *>(&remote_addr_),
                sizeof(remote_addr_)) != 0)
    {
      gzerr << "[PoleMavlinkPosePlugin] TCP connect failed to " << tcp_addr_
            << ":" << tcp_port_ << ", will retry\n";
      CloseSocket();
      return false;
    }

    gzmsg << "[PoleMavlinkPosePlugin] TCP connected to " << tcp_addr_ << ":"
          << tcp_port_ << "\n";
    return true;
  }

  void CloseSocket()
  {
    if (socket_fd_ >= 0)
    {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void ApplyPendingPositionCommand(
      gz::sim::EntityComponentManager &_ecm)
  {
    gz::math::Vector3d target_ned;
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      if (!has_pending_position_)
        return;
      target_ned = pending_position_ned_;
      has_pending_position_ = false;
    }

    const gz::math::Vector3d target_world = NedToWorld(target_ned);
    gz::math::Pose3d pose = gz::sim::worldPose(model_entity_, _ecm);
    pose.Pos() = target_world;
    model_.SetWorldPoseCmd(_ecm, pose);

    gz::sim::Link link(link_entity_);
    link.SetLinearVelocity(_ecm, gz::math::Vector3d::Zero);
    link.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
    clear_velocity_commands_ = true;
  }

  static gz::math::Vector3d WorldToNed(
      const gz::math::Vector3d &_world)
  {
    // Match the ArduPilot Gazebo plugins: Gazebo world xyz is N, -E, -D.
    return gz::math::Vector3d(_world.X(), -_world.Y(), -_world.Z());
  }

  static gz::math::Vector3d NedToWorld(
      const gz::math::Vector3d &_ned)
  {
    return gz::math::Vector3d(_ned.X(), -_ned.Y(), -_ned.Z());
  }

  void SendOdometry(
      uint64_t _time_usec,
      const gz::math::Vector3d &_position_ned,
      const gz::math::Vector3d &_velocity_ned)
  {
    mavlink_message_t message;
    mavlink_odometry_t odometry{};

    odometry.time_usec = _time_usec;
    odometry.frame_id = MAV_FRAME_LOCAL_NED;
    odometry.child_frame_id = MAV_FRAME_LOCAL_NED;
    odometry.x = static_cast<float>(_position_ned.X());
    odometry.y = static_cast<float>(_position_ned.Y());
    odometry.z = static_cast<float>(_position_ned.Z());
    odometry.vx = static_cast<float>(_velocity_ned.X());
    odometry.vy = static_cast<float>(_velocity_ned.Y());
    odometry.vz = static_cast<float>(_velocity_ned.Z());
    odometry.q[0] = 1.0f;
    odometry.q[1] = 0.0f;
    odometry.q[2] = 0.0f;
    odometry.q[3] = 0.0f;
    odometry.rollspeed = 0.0f;
    odometry.pitchspeed = 0.0f;
    odometry.yawspeed = 0.0f;
    odometry.pose_covariance[0] = std::numeric_limits<float>::quiet_NaN();
    odometry.velocity_covariance[0] =
        std::numeric_limits<float>::quiet_NaN();
    odometry.estimator_type = MAV_ESTIMATOR_TYPE_NAIVE;
    odometry.quality = 100;

    mavlink_msg_odometry_encode(system_id_, component_id_, &message,
                                 &odometry);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
    const ssize_t sent = send(socket_fd_, buffer, length, MSG_NOSIGNAL);
    if (sent != static_cast<ssize_t>(length))
    {
      gzerr << "[PoleMavlinkPosePlugin] TCP send failed, closing socket and "
               "retrying later\n";
      CloseSocket();
    }
  }

  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string link_name_{"pole"};
  bool warned_missing_link_{false};

  std::string tcp_addr_{"127.0.0.1"};
  int tcp_port_{-1};
  double send_rate_hz_{50.0};
  double last_send_time_{-1.0};
  double last_connect_attempt_time_{-1.0};
  std::string ros_target_topic_{"/pendulum_pole/set_position_ned"};
  std::string ros_frame_id_{"local_ned"};
  gz::math::Vector3d odom_point_offset_link_{0, 0, 0};
  uint8_t system_id_{42};
  uint8_t component_id_{MAV_COMP_ID_ONBOARD_COMPUTER};

  int socket_fd_{-1};
  sockaddr_in remote_addr_{};
  bool clear_velocity_commands_{false};

  std::shared_ptr<rclcpp::Context> ros_context_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      target_position_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread ros_thread_;
  std::mutex target_mutex_;
  bool has_pending_position_{false};
  gz::math::Vector3d pending_position_ned_{0, 0, 0};
};

}  // namespace gazebo

GZ_ADD_PLUGIN(
    gazebo::PoleMavlinkPosePlugin,
    gz::sim::System,
    gazebo::PoleMavlinkPosePlugin::ISystemConfigure,
    gazebo::PoleMavlinkPosePlugin::ISystemPreUpdate,
    gazebo::PoleMavlinkPosePlugin::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gazebo::PoleMavlinkPosePlugin, "PoleMavlinkPosePlugin")
// Preserve the original SDF instance name under Gazebo Sim's class lookup.
GZ_ADD_PLUGIN_ALIAS(gazebo::PoleMavlinkPosePlugin, "pole_mavlink_pose_plugin")
