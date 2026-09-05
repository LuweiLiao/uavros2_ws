#include "HelloModelPlugin.hh"

#include <gz/plugin/Register.hh>

namespace gazebo
{

void HelloModelPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  model_entity_ = _entity;
  model_ = gz::sim::Model(model_entity_);
  // The ROS 1 plugin acted on the model's first link.  Keep that generic
  // responsibility rather than introducing a vehicle-specific link name.
  const auto links = model_.Links(_ecm);
  if (!links.empty())
    link_entity_ = links.front();
  gzdbg << "HelloModelPlugin\n";
}

void HelloModelPlugin::PreUpdate(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm)
{
  if (link_entity_ == gz::sim::kNullEntity)
    return;
  gz::sim::Link(link_entity_).SetLinearVelocity(
      _ecm, gz::math::Vector3d(0.3, 0.0, 0.0));
}

}  // namespace gazebo

GZ_ADD_PLUGIN(
    gazebo::HelloModelPlugin,
    gz::sim::System,
    gazebo::HelloModelPlugin::ISystemConfigure,
    gazebo::HelloModelPlugin::ISystemPreUpdate)
