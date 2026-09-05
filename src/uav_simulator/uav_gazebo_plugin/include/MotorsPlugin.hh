#pragma once

#include <memory>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace turning_direction
{
constexpr int CCW = 1;
constexpr int CW = -1;
}  // namespace turning_direction

enum class MotorType
{
  kVelocity,
  kPosition,
  kForce
};

namespace gazebo
{

class MotorsPlugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override;

  void PreUpdate(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager &_ecm) override;

private:
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity joint_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};

  std::string joint_name_;
  std::string link_name_;
  int motor_number_{0};
  int turning_direction_{turning_direction::CCW};
  MotorType motor_type_{MotorType::kVelocity};

  bool publish_speed_{false};
  bool publish_position_{false};
  bool publish_force_{false};
  std::string command_sub_topic_{"command/motor_speed"};
  std::string wind_speed_sub_topic_{"wind_speed"};
  std::string motor_speed_pub_topic_{"motor_speed"};
  std::string motor_position_pub_topic_;
  std::string motor_force_pub_topic_;

  double max_force_{0.0};
  double max_rot_velocity_{838.0};
  double moment_constant_{0.016};
  double motor_constant_{8.54858e-6};
  double ref_motor_input_{0.0};
  double rolling_moment_coefficient_{1.0e-6};
  double rotor_drag_coefficient_{1.0e-4};
  double rotor_velocity_slowdown_sim_{10.0};
  double time_constant_down_{1.0 / 40.0};
  double time_constant_up_{1.0 / 80.0};
  gz::math::Vector3d wind_speed_W_;
};

}  // namespace gazebo
