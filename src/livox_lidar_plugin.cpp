#include <gz/plugin/Register.hh>
#include <iostream>
#include "livox_lidar_plugin.hpp"
 
// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    sample_system::SampleSystem,
    gz::sim::System,
    sample_system::SampleSystem::ISystemConfigure,
    sample_system::SampleSystem::ISystemPostUpdate)

using namespace sample_system;
 
SampleSystem::SampleSystem()
{
}
 
SampleSystem::~SampleSystem()
{
}

void SampleSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  // std::cerr is independent of Gazebo's verbosity filtering.
  std::cerr << "[livox_lidar_gz_sensor] SampleSystem started on entity "
            << _entity << std::endl;
}
 
void SampleSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gzmsg << "SampleSystem::PostUpdate" << std::endl;
}
