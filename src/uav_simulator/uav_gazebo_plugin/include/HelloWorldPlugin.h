#pragma once

#include <memory>

#include <gz/sim/System.hh>

namespace gazebo
{

class HelloWorldPlugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
public:
  HelloWorldPlugin();

  void Configure(
      const gz::sim::Entity & /*_entity*/,
      const std::shared_ptr<const sdf::Element> & /*_sdf*/,
      gz::sim::EntityComponentManager & /*_ecm*/,
      gz::sim::EventManager & /*_eventMgr*/) override;
};

}  // namespace gazebo
