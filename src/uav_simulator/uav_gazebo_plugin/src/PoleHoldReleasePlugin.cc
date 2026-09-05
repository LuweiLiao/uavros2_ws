/*
 * Gazebo Sim port of the ROS 1 PoleHoldReleasePlugin.
 *
 * The source filename, class name, shared-library name, SDF keys and ROS
 * topic contract are deliberately unchanged.  Only the Gazebo Classic and
 * ROS 1 API boundaries are adapted to Gazebo Sim and ROS 2.  ROS callbacks
 * only enqueue commands; all physics operations remain in PreUpdate, where
 * the Gazebo Sim ECM is valid and thread-safe.
 */

#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace gazebo
{

class PoleHoldReleasePlugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
public:
  PoleHoldReleasePlugin() = default;

  ~PoleHoldReleasePlugin() override
  {
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
    unlock_topic_ = _sdf->Get(
        "unlockTopic", std::string("/pendulum_pole/unlock")).first;
    set_position_topic_ = _sdf->Get(
        "setPositionTopic",
        std::string("/pendulum_pole/set_position_ned")).first;
    pos_p_ = _sdf->Get("posP", pos_p_).first;
    pos_d_ = _sdf->Get("posD", pos_d_).first;
    rot_p_ = _sdf->Get("rotP", rot_p_).first;
    rot_d_ = _sdf->Get("rotD", rot_d_).first;
    max_force_ = _sdf->Get("maxForce", max_force_).first;
    max_torque_ = _sdf->Get("maxTorque", max_torque_).first;

    ResolveEntities(_ecm);
    InitializeRos();

    gzmsg << "[PoleHoldReleasePlugin] Holding ["
          << gz::sim::scopedName(model_entity_, _ecm) << "::" << link_name_
          << "] until " << unlock_topic_ << " receives true\n";
    gzmsg << "[PoleHoldReleasePlugin] Updating hold position from ROS topic ["
          << set_position_topic_ << "]\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!ResolveEntities(_ecm))
      return;

    // Link::Set*Velocity creates a one-step Gazebo command component.  Remove
    // our previous command before the next physics step so an unlock request
    // does not accidentally keep the pole frozen forever.
    if (clear_velocity_commands_)
    {
      _ecm.RemoveComponent<gz::sim::components::WorldLinearVelocityCmd>(
          link_entity_);
      _ecm.RemoveComponent<gz::sim::components::WorldAngularVelocityCmd>(
          link_entity_);
      _ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(
          link_entity_);
      _ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(
          link_entity_);
      clear_velocity_commands_ = false;
    }

    ProcessPendingCommands(_ecm);

    bool unlocked = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      unlocked = unlocked_;
    }
    if (unlocked)
      return;

    gz::sim::Link link(link_entity_);
    const auto pose = link.WorldPose(_ecm);
    if (!pose)
      return;

    const auto velocity = link.WorldLinearVelocity(_ecm);
    const auto angular_velocity = link.WorldAngularVelocity(_ecm);
    const gz::math::Vector3d linear_vel = velocity.value_or(
        gz::math::Vector3d::Zero);
    const gz::math::Vector3d angular_vel = angular_velocity.value_or(
        gz::math::Vector3d::Zero);

    const auto gravity = gz::sim::World(gz::sim::worldEntity(
        model_entity_, _ecm)).Gravity(_ecm);
    const gz::math::Vector3d gravity_vector = gravity.value_or(
        gz::math::Vector3d(0.0, 0.0, -9.8));

    const gz::math::Vector3d position_error = hold_pose_.Pos() - pose->Pos();
    gz::math::Vector3d force = pos_p_ * position_error - pos_d_ * linear_vel;

    // Match the ROS 1 plugin: cancel gravity while the pole is held, then
    // use the configured PD term to remove any remaining drift.
    force += -mass_ * gravity_vector;

    const gz::math::Vector3d hold_rpy = hold_pose_.Rot().Euler();
    const gz::math::Vector3d current_rpy = pose->Rot().Euler();
    const gz::math::Vector3d rotation_error(
        NormalizeAngle(hold_rpy.X() - current_rpy.X()),
        NormalizeAngle(hold_rpy.Y() - current_rpy.Y()),
        NormalizeAngle(hold_rpy.Z() - current_rpy.Z()));
    const gz::math::Vector3d torque =
        rot_p_ * rotation_error - rot_d_ * angular_vel;

    link.AddWorldWrench(_ecm, LimitVector(force, max_force_),
                        LimitVector(torque, max_torque_));
  }

  void Reset(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!ResolveEntities(_ecm))
      return;

    const auto pose = gz::sim::Link(link_entity_).WorldPose(_ecm);
    if (pose)
      hold_pose_ = *pose;

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      unlocked_ = false;
      pending_unlock_ = false;
      pending_position_ = false;
    }
    ZeroVelocities(_ecm);
  }

private:
  bool ResolveEntities(gz::sim::EntityComponentManager &_ecm)
  {
    if (link_entity_ == gz::sim::kNullEntity)
      link_entity_ = model_.LinkByName(_ecm, link_name_);

    if (link_entity_ == gz::sim::kNullEntity)
    {
      if (!warned_missing_link_)
      {
        gzerr << "[PoleHoldReleasePlugin] Link [" << link_name_
              << "] not found in model entity [" << model_entity_ << "]\n";
        warned_missing_link_ = true;
      }
      return false;
    }

    gz::sim::Link link(link_entity_);
    link.EnableVelocityChecks(_ecm);

    if (!have_hold_pose_)
    {
      const auto pose = link.WorldPose(_ecm);
      if (!pose)
        return false;
      hold_pose_ = *pose;
      have_hold_pose_ = true;
    }

    if (mass_ <= 0.0)
    {
      const auto inertial = link.WorldInertial(_ecm);
      if (inertial)
        mass_ = inertial->MassMatrix().Mass();
    }
    return true;
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
          "pole_hold_release_plugin", node_options);

      const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
      unlock_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
          unlock_topic_, qos,
          [this](std_msgs::msg::Bool::ConstSharedPtr _message)
          {
            std::lock_guard<std::mutex> lock(command_mutex_);
            pending_unlock_ = true;
            requested_unlocked_ = _message->data;
          });

      set_position_sub_ =
          ros_node_->create_subscription<geometry_msgs::msg::PointStamped>(
              set_position_topic_, qos,
              [this](geometry_msgs::msg::PointStamped::ConstSharedPtr _message)
              {
                std::lock_guard<std::mutex> lock(command_mutex_);
                pending_position_ = true;
                pending_target_ned_ = gz::math::Vector3d(
                    _message->point.x, _message->point.y,
                    _message->point.z);
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
      gzerr << "[PoleHoldReleasePlugin] ROS 2 initialization failed: "
            << error.what() << "\n";
      ShutdownRos();
    }
  }

  void ShutdownRos()
  {
    if (executor_)
      executor_->cancel();
    if (ros_context_ && ros_context_->is_valid())
      ros_context_->shutdown("PoleHoldReleasePlugin shutdown");
    if (ros_thread_.joinable())
      ros_thread_.join();
    if (executor_ && ros_node_)
      executor_->remove_node(ros_node_);
    unlock_sub_.reset();
    set_position_sub_.reset();
    ros_node_.reset();
    executor_.reset();
    ros_context_.reset();
  }

  void ProcessPendingCommands(gz::sim::EntityComponentManager &_ecm)
  {
    bool have_unlock = false;
    bool requested_unlocked = false;
    bool have_position = false;
    gz::math::Vector3d target_ned;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      have_unlock = pending_unlock_;
      requested_unlocked = requested_unlocked_;
      pending_unlock_ = false;
      have_position = pending_position_;
      target_ned = pending_target_ned_;
      pending_position_ = false;
    }

    if (have_position)
    {
      const gz::math::Vector3d target_world = NedToWorld(target_ned);
      hold_pose_.Pos() = target_world;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        unlocked_ = false;
      }
      model_.SetWorldPoseCmd(_ecm, hold_pose_);
      ZeroVelocities(_ecm);

      gzmsg << "[PoleHoldReleasePlugin] Hold position updated from NED "
            << target_ned << " to Gazebo world " << target_world << "\n";
    }

    if (have_unlock)
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      unlocked_ = requested_unlocked;
      if (requested_unlocked)
        ZeroVelocities(_ecm);
    }
  }

  void ZeroVelocities(gz::sim::EntityComponentManager &_ecm)
  {
    if (link_entity_ == gz::sim::kNullEntity)
      return;
    gz::sim::Link link(link_entity_);
    link.SetLinearVelocity(_ecm, gz::math::Vector3d::Zero);
    link.SetAngularVelocity(_ecm, gz::math::Vector3d::Zero);
    clear_velocity_commands_ = true;
  }

  static gz::math::Vector3d NedToWorld(
      const gz::math::Vector3d &_ned)
  {
    // Match the ArduPilot Gazebo plugins: Gazebo world xyz is N, -E, -D.
    return gz::math::Vector3d(_ned.X(), -_ned.Y(), -_ned.Z());
  }

  static double NormalizeAngle(double _angle)
  {
    constexpr double kPi = 3.1415926535897932384626433832795;
    while (_angle > kPi)
      _angle -= 2.0 * kPi;
    while (_angle < -kPi)
      _angle += 2.0 * kPi;
    return _angle;
  }

  static gz::math::Vector3d LimitVector(
      const gz::math::Vector3d &_vector, double _limit)
  {
    if (_limit <= 0.0 || _vector.Length() <= _limit)
      return _vector;
    return _vector.Normalized() * _limit;
  }

  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};
  std::string link_name_{"pole"};

  gz::math::Pose3d hold_pose_;
  bool have_hold_pose_{false};
  bool warned_missing_link_{false};
  bool clear_velocity_commands_{false};
  double mass_{0.0};
  double pos_p_{80.0};
  double pos_d_{18.0};
  double rot_p_{2.0};
  double rot_d_{0.4};
  double max_force_{200.0};
  double max_torque_{20.0};

  std::string unlock_topic_{"/pendulum_pole/unlock"};
  std::string set_position_topic_{"/pendulum_pole/set_position_ned"};

  std::mutex command_mutex_;
  bool unlocked_{false};
  bool pending_unlock_{false};
  bool requested_unlocked_{false};
  bool pending_position_{false};
  gz::math::Vector3d pending_target_ned_;

  std::shared_ptr<rclcpp::Context> ros_context_;
  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr unlock_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
      set_position_sub_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread ros_thread_;
};

}  // namespace gazebo

GZ_ADD_PLUGIN(
    gazebo::PoleHoldReleasePlugin,
    gz::sim::System,
    gazebo::PoleHoldReleasePlugin::ISystemConfigure,
    gazebo::PoleHoldReleasePlugin::ISystemPreUpdate,
    gazebo::PoleHoldReleasePlugin::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gazebo::PoleHoldReleasePlugin, "PoleHoldReleasePlugin")
// Gazebo Classic treated the SDF name as an instance label.  Gazebo Sim
// resolves it as a class alias, so preserve the original SDF name verbatim.
GZ_ADD_PLUGIN_ALIAS(gazebo::PoleHoldReleasePlugin, "pole_hold_release_plugin")
