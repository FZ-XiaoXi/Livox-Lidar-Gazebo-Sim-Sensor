#ifndef __LIVOX_LIDAR_PLUGIN_HPP__
#define __LIVOX_LIDAR_PLUGIN_HPP__

#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
 
namespace sample_system
{
  class SampleSystem:
    // This class is a system.
    public gz::sim::System,
    // Configure is called once when Gazebo attaches the plugin to an entity.
    public gz::sim::ISystemConfigure,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPostUpdate
  {
    public: SampleSystem();
 
    public: ~SampleSystem() override;

    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_element,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventManager) override;
 
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
  };
}

#endif  //__LIVOX_LIDAR_PLUGIN_HPP__
