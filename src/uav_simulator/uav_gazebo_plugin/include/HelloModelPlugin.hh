#pragma once

#include <memory>

#include <gz/math/Vector3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

namespace gazebo
{

class HelloModelPlugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> & /*_sdf*/,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override;

  void PreUpdate(
      const gz::sim::UpdateInfo & /*_info*/,
      gz::sim::EntityComponentManager &_ecm) override;

private:
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::sim::Model model_{gz::sim::kNullEntity};
};

}  // namespace gazebo
