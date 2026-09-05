#include "MotorsPlugin.hh"

#include <string>

#include <gz/plugin/Register.hh>

namespace gazebo
{

void MotorsPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  model_entity_ = _entity;
  model_ = gz::sim::Model(model_entity_);

  joint_name_ = _sdf->Get("jointName", joint_name_).first;
  link_name_ = _sdf->Get("linkName", link_name_).first;
  motor_number_ = _sdf->Get("motorNumber", motor_number_).first;

  const std::string direction = _sdf->Get(
      "turningDirection", std::string("ccw")).first;
  if (direction == "cw")
    turning_direction_ = turning_direction::CW;
  else if (direction == "ccw")
    turning_direction_ = turning_direction::CCW;
  else
    gzerr << "[MotorsPlugin] Please only use 'cw' or 'ccw' as turningDirection.\n";

  const std::string type = _sdf->Get(
      "motorType", std::string("velocity")).first;
  if (type == "velocity")
    motor_type_ = MotorType::kVelocity;
  else if (type == "position")
    motor_type_ = MotorType::kPosition;
  else if (type == "force")
    motor_type_ = MotorType::kForce;
  else
    gzerr << "[MotorsPlugin] Please only use 'velocity', 'position' or 'force' "
             "as motorType.\n";

  command_sub_topic_ = _sdf->Get(
      "commandSubTopic", command_sub_topic_).first;
  wind_speed_sub_topic_ = _sdf->Get(
      "windSpeedSubTopic", wind_speed_sub_topic_).first;
  motor_speed_pub_topic_ = _sdf->Get(
      "motorSpeedPubTopic", motor_speed_pub_topic_).first;
  if (_sdf->HasElement("motorPositionPubTopic"))
  {
    publish_position_ = true;
    motor_position_pub_topic_ = _sdf->Get(
        "motorPositionPubTopic", motor_position_pub_topic_).first;
  }
  if (_sdf->HasElement("motorForcePubTopic"))
  {
    publish_force_ = true;
    motor_force_pub_topic_ = _sdf->Get(
        "motorForcePubTopic", motor_force_pub_topic_).first;
  }

  max_rot_velocity_ = _sdf->Get(
      "maxRotVelocity", max_rot_velocity_).first;
  motor_constant_ = _sdf->Get("motorConstant", motor_constant_).first;
  moment_constant_ = _sdf->Get("momentConstant", moment_constant_).first;
  rotor_drag_coefficient_ = _sdf->Get(
      "rotorDragCoefficient", rotor_drag_coefficient_).first;
  rolling_moment_coefficient_ = _sdf->Get(
      "rollingMomentCoefficient", rolling_moment_coefficient_).first;
  time_constant_up_ = _sdf->Get("timeConstantUp", time_constant_up_).first;
  time_constant_down_ = _sdf->Get(
      "timeConstantDown", time_constant_down_).first;
  rotor_velocity_slowdown_sim_ = _sdf->Get(
      "rotorVelocitySlowdownSim", rotor_velocity_slowdown_sim_).first;

  if (!joint_name_.empty())
    joint_entity_ = model_.JointByName(_ecm, joint_name_);
  if (!link_name_.empty())
    link_entity_ = model_.LinkByName(_ecm, link_name_);

  if (joint_entity_ == gz::sim::kNullEntity)
    gzerr << "[MotorsPlugin] Couldn't find specified joint \"" << joint_name_
          << "\".\n";
  if (link_entity_ == gz::sim::kNullEntity)
    gzerr << "[MotorsPlugin] Couldn't find specified link \"" << link_name_
          << "\".\n";

  gzdbg << "[MotorsPlugin] motorNumber=" << motor_number_
        << " commandSubTopic=" << command_sub_topic_
        << " motorType=" << type << " turningDirection=" << direction
        << "\n";
}

void MotorsPlugin::PreUpdate(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm)
{
  // The ROS 1 implementation's active update behavior is intentionally kept
  // verbatim: it applies a small +X model velocity.  Its ROS/Gazebo topic
  // setup was commented out, so no new command transport is invented here.
  if (link_entity_ == gz::sim::kNullEntity)
    return;
  gz::sim::Link(link_entity_).SetLinearVelocity(
      _ecm, gz::math::Vector3d(0.3, 0.0, 0.0));
}

}  // namespace gazebo

GZ_ADD_PLUGIN(
    gazebo::MotorsPlugin,
    gz::sim::System,
    gazebo::MotorsPlugin::ISystemConfigure,
    gazebo::MotorsPlugin::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(gazebo::MotorsPlugin, "MotorsPlugin")
