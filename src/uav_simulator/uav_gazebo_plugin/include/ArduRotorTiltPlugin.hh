/*
 * Gazebo Sim port of the ROS 1 ArduRotorTiltPlugin.
 *
 * The original class, source/header paths, library name, SDF keys, UDP
 * protocol, ROS topic names and actuator ordering are kept unchanged.  Only
 * the Gazebo Classic and ROS 1 API boundaries are adapted.
 */
#ifndef GAZEBO_PLUGINS_ARDUROTOR_TILT_PLUGIN_HH_
#define GAZEBO_PLUGINS_ARDUROTOR_TILT_PLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gazebo
{

class ArduRotorTiltPlugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
public:
  ArduRotorTiltPlugin();
  ~ArduRotorTiltPlugin() override;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

  void Reset(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

private:
  class Private;

  bool ResolveEntities(gz::sim::EntityComponentManager &_ecm);
  void ReceiveMotorCommand();
  void PublishActuators();
  void SendState(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm);
  void InitializeRos();
  void ShutdownRos();

  std::unique_ptr<Private> dataPtr;
};

}  // namespace gazebo

#endif  // GAZEBO_PLUGINS_ARDUROTOR_TILT_PLUGIN_HH_
