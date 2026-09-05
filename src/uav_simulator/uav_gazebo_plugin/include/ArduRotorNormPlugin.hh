/*
 * Gazebo Sim port of the ROS 1 ArduRotorNormPlugin.
 *
 * The public contract is intentionally unchanged: the class, source file,
 * shared-library name and SDF keys remain ArduRotorNormPlugin /
 * libArduRotorNormPlugin.so.  Only the Gazebo API boundary is adapted.
 */
#ifndef GAZEBO_PLUGINS_ARDUROTORNORMPLUGIN_HH_
#define GAZEBO_PLUGINS_ARDUROTORNORMPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gazebo
{

/// Bridge the legacy ArduPilot Gazebo UDP protocol to the RotorS actuator
/// topic used by the original tsduav_quad model.
class ArduRotorNormPlugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
public:
  ArduRotorNormPlugin();
  ~ArduRotorNormPlugin() override;

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

  /// Resolve nested links/sensors after Gazebo has populated the ECM.
  bool ResolveEntities(gz::sim::EntityComponentManager &_ecm);

  /// Receive and publish one ArduPilot servo packet.
  void ReceiveMotorCommand();
  void PublishMotorCommand();

  /// Send the original 17-double FDM packet back to SITL.
  void SendState(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm);

  std::unique_ptr<Private> dataPtr;
};

}  // namespace gazebo

#endif  // GAZEBO_PLUGINS_ARDUROTORNORMPLUGIN_HH_
