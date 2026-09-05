/*
 * Gazebo Sim port of the ROS 1 RotorS GazeboMotorModel.
 *
 * The class, SDF keys, source filename and shared-library name remain the
 * original RotorS contract.  The ROS 1 Gazebo transport message is replaced
 * by the native Gazebo Sim gz.msgs.Actuators message at the API boundary.
 */
#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H

#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/float.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace rotors_gazebo_plugins
{

class GazeboMotorModel final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  GazeboMotorModel();
  ~GazeboMotorModel() override;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

private:
  class Private;
  std::unique_ptr<Private> dataPtr;
};

}  // namespace rotors_gazebo_plugins

#endif  // ROTORS_GAZEBO_PLUGINS_MOTOR_MODELS_H
