#include "HelloWorldPlugin.h"

#include <cstdio>

#include <gz/plugin/Register.hh>

namespace gazebo
{

HelloWorldPlugin::HelloWorldPlugin()
{
  std::printf("Hello World!\n");
}

void HelloWorldPlugin::Configure(
    const gz::sim::Entity & /*_entity*/,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    gz::sim::EntityComponentManager & /*_ecm*/,
    gz::sim::EventManager & /*_eventMgr*/)
{
}

}  // namespace gazebo

GZ_ADD_PLUGIN(
    gazebo::HelloWorldPlugin,
    gz::sim::System,
    gazebo::HelloWorldPlugin::ISystemConfigure)
