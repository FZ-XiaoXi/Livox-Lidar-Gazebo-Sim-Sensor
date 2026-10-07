#ifndef __LIVOX_LIDAR_SYSTEM_HPP__
#define __LIVOX_LIDAR_SYSTEM_HPP__

#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include "livox_lidar_sensor.hpp"

 
namespace livox_lidar_system
{
  class LivoxLidarSystem:
    // This class is a system.
    public gz::sim::System,
    // Configure is called once when Gazebo attaches the plugin to an entity.
    public gz::sim::ISystemConfigure,

    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemPreUpdate,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPostUpdate
  {
    public: LivoxLidarSystem();
 
    public: ~LivoxLidarSystem() override;

    public: void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_element,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventManager) override;
    
    // During PreUpdate, check for new sensors that were inserted
    // into simulation and create more components as needed.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;


    // During PostUpdate, update the known sensors and publish their data.
    // Also remove sensors that have been deleted.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Remove custom sensors if their entities have been removed from
    /// simulation.
    /// \param[in] _ecm Immutable reference to ECM.
    private: void RemoveSensorEntities(
        const gz::sim::EntityComponentManager &_ecm);

    /// \brief A map of custom entities to their sensors
    private: std::unordered_map<gz::sim::Entity,
        std::shared_ptr<livox_lidar_sensor::LivoxLidarSensor>> entitySensorMap_;

    /// \brief The entity that owns this system.
    private: gz::sim::Entity ownerEntity_;

    private: bool IsOwnedBy(
      gz::sim::Entity _sensor,
      gz::sim::Entity _owner,
      const gz::sim::EntityComponentManager &_ecm) const;
  };
}

#endif  //__LIVOX_LIDAR_SYSTEM_HPP__
