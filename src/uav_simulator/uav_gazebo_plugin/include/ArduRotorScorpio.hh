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
#ifndef GAZEBO_PLUGINS_ARDUROTOR_SCORPIO_HH_
#define GAZEBO_PLUGINS_ARDUROTOR_SCORPIO_HH_
#include <memory>
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>

namespace gazebo {
class ArduRotorScorpio final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemReset {
public:
    ArduRotorScorpio();
    ~ArduRotorScorpio() override;
    void Configure(const gz::sim::Entity &,
        const std::shared_ptr<const sdf::Element> &,
        gz::sim::EntityComponentManager &, gz::sim::EventManager &) override;
    void PreUpdate(const gz::sim::UpdateInfo &,
                   gz::sim::EntityComponentManager &) override;
    void Reset(const gz::sim::UpdateInfo &,
               gz::sim::EntityComponentManager &) override;
private:
    class Private;
    bool ResolveEntities(gz::sim::EntityComponentManager &);
    void ApplyMotorForces(double);
    void ResetPIDs();
    void ReceiveMotorCommand();
    void ReceiveSocketCAN();
    bool InitSocketCAN();
    bool InitArduPilotSockets(sdf::ElementPtr) const;
    void SendState(const gz::sim::UpdateInfo &,
                   const gz::sim::EntityComponentManager &) const;
    std::unique_ptr<Private> dataPtr;
    gz::math::Pose3d modelXYZToAirplaneXForwardZDown, gazeboXYZToNED;
};
}  // namespace gazebo
#endif
