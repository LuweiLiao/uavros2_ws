/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Gazebo Sim port of the ROS 1 ArduRotorTiltQuadcopter.
 *
 * The public contract is intentionally unchanged: the class, source file,
 * shared-library name and SDF keys remain ArduRotorTiltQuadcopter /
 * libArduRotorTiltQuadcopter.so.  Only the Gazebo API boundary is adapted.
 */
#ifndef GAZEBO_PLUGINS_ARDUROTORTILTQUADCOPTER_HH_
#define GAZEBO_PLUGINS_ARDUROTORTILTQUADCOPTER_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gazebo
{

/// Bridge the legacy ArduPilot Gazebo UDP protocol to the RotorS actuator
/// topic used by the original tilt_quadcopter model.
class ArduRotorTiltQuadcopter final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
public:
  ArduRotorTiltQuadcopter();
  ~ArduRotorTiltQuadcopter() override;

  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

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

#endif  // GAZEBO_PLUGINS_ARDUROTORTILTQUADCOPTER_HH_
