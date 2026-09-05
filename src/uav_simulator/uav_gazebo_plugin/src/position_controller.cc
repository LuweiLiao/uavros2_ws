/*
 * Gazebo Sim / ROS 2 port of the original position_controller plugin.
 *
 * The ROS 1 package defines this class entirely in position_controller.cc;
 * there is no corresponding header to migrate.  Keep that layout, class
 * name, target name, SDF keys, topic name, and zero-position PID behavior.
 * Only the Gazebo Classic and ROS 1 API boundaries are adapted here.
 */

#ifndef _POSITION_PLUGIN_HH_
#define _POSITION_PLUGIN_HH_

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gz/math/PID.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace gazebo
{

/// \brief A plugin to control joint position.
class position_controller final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
public:
  position_controller() = default;

  ~position_controller() override
  {
    ShutdownRos();
  }

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    model = gz::sim::Model(_entity);
    if (!model.Valid(_ecm))
    {
      gzerr << "Invalid model, position plugin not loaded\n";
      return;
    }

    // Match the ROS 1 safety check before configuring the controller.
    if (model.JointCount(_ecm) == 0u)
    {
      gzerr << "Invalid joint count, position plugin not loaded\n";
      return;
    }

    // Keep the original SDF element names.  Zero is the only safe fallback
    // for a missing gain; all original model configurations are expected to
    // provide these values explicitly.
    joint_name_ori = _sdf->Get("jointname", std::string()).first;
    const double p_gain = _sdf->Get("p_gain", 0.0).first;
    const double i_gain = _sdf->Get("i_gain", 0.0).first;
    const double d_gain = _sdf->Get("d_gain", 0.0).first;

    model_name = model.Name(_ecm);
    joint_name = gz::sim::scopedName(_entity, _ecm) + "::" + joint_name_ori;

    gzmsg << "We find the joint [" << joint_name << "]\n";
    gzmsg << "\n The model's name is [" << model_name << "]\n";

    pid.Init(p_gain, i_gain, d_gain);
    target_position = 0.0;

    if (!ResolveJoint(_ecm))
    {
      gzerr << "Position controller could not find joint ["
            << joint_name_ori << "] in model [" << model_name << "]\n";
      return;
    }

    InitializeRos();
    configured = true;
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!configured || _info.paused)
      return;

    if (!ResolveJoint(_ecm))
      return;

    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0.0)
      return;

    const auto *position = _ecm.Component<
        gz::sim::components::JointPosition>(joint);
    if (position == nullptr || position->Data().empty())
      return;

    // gazebo::common::PID and gz::math::PID share the same error convention:
    // current state minus target.  The returned command is applied as the
    // joint force / torque, matching JointController::SetPositionTarget.
    const double error = position->Data().front() - target_position;
    const double force = pid.Update(error, std::chrono::duration<double>(dt));
    gz::sim::Joint(joint).SetForce(_ecm, std::vector<double>{force});
  }

  void Reset(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager & /*_ecm*/) override
  {
    pid.Reset();
    target_position = 0.0;
  }

  /// \brief Set the position of the joint.
  /// \param[in] _pos New target position.
  void SetPosition(const double &_pos)
  {
    target_position = _pos;
  }

  /// \brief Handle an incoming message from ROS.
  ///
  /// The corresponding call to SetPosition is commented out in the ROS 1
  /// source, so receiving this preserved topic intentionally remains a no-op.
  void OnRosMsg(const std_msgs::msg::Float32::ConstSharedPtr &_msg)
  {
    (void)_msg;
    // this->SetPosition(_msg->data);
  }

private:
  bool ResolveJoint(gz::sim::EntityComponentManager &_ecm)
  {
    if (joint != gz::sim::kNullEntity)
      return true;

    joint = model.JointByName(_ecm, joint_name_ori);
    if (joint == gz::sim::kNullEntity)
    {
      // Preserve support for the scoped joint name used by the ROS 1
      // JointController.  This fallback does not add a new SDF parameter.
      auto matches = gz::sim::entitiesFromScopedName(
          joint_name_ori, _ecm, model.Entity());
      for (const auto entity : matches)
      {
        if (_ecm.EntityHasComponentType(
            entity, gz::sim::components::Joint::typeId))
        {
          joint = entity;
          break;
        }
      }
    }

    if (joint == gz::sim::kNullEntity)
      return false;

    gz::sim::Joint joint_handle(joint);
    joint_handle.EnablePositionCheck(_ecm);
    return true;
  }

  void InitializeRos()
  {
    try
    {
      ros_context = std::make_shared<rclcpp::Context>();
      ros_context->init(0, nullptr);

      rclcpp::NodeOptions options;
      options.context(ros_context);
      ros_node = std::make_shared<rclcpp::Node>(
          joint_name_ori + "_node", options);

      const std::string topic =
          "/" + model_name + "/" + joint_name_ori + "/pos_cmd";
      ros_sub = ros_node->create_subscription<std_msgs::msg::Float32>(
          topic, rclcpp::QoS(rclcpp::KeepLast(1)),
          [this](std_msgs::msg::Float32::ConstSharedPtr message)
          {
            OnRosMsg(message);
          });

      rclcpp::ExecutorOptions executor_options;
      executor_options.context = ros_context;
      ros_executor = std::make_unique<
          rclcpp::executors::SingleThreadedExecutor>(executor_options);
      ros_executor->add_node(ros_node);
      ros_thread = std::thread([this]()
      {
        if (ros_executor)
          ros_executor->spin();
      });
    }
    catch (const std::exception &error)
    {
      gzerr << "[position_controller] ROS 2 initialization failed: "
            << error.what() << "\n";
      ShutdownRos();
    }
  }

  void ShutdownRos()
  {
    if (ros_executor)
      ros_executor->cancel();
    if (ros_context && ros_context->is_valid())
      ros_context->shutdown("position_controller shutdown");
    if (ros_thread.joinable())
      ros_thread.join();
    if (ros_executor && ros_node)
      ros_executor->remove_node(ros_node);

    ros_sub.reset();
    ros_node.reset();
    ros_executor.reset();
    ros_context.reset();
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity joint{gz::sim::kNullEntity};
  gz::math::PID pid;

  std::shared_ptr<rclcpp::Context> ros_context;
  std::shared_ptr<rclcpp::Node> ros_node;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ros_sub;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor;
  std::thread ros_thread;

  bool configured{false};
  double target_position{0.0};

public:
  std::string joint_name;
  std::string joint_name_ori;

private:
  std::string model_name;
};

}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::position_controller, gz::sim::System,
  gazebo::position_controller::ISystemConfigure,
  gazebo::position_controller::ISystemPreUpdate,
  gazebo::position_controller::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gazebo::position_controller, "position_controller")

#endif  // _POSITION_PLUGIN_HH_
