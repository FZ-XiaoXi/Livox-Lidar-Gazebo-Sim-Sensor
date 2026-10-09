#ifndef __LIVOX_LIDAR_SYSTEM_HPP__
#define __LIVOX_LIDAR_SYSTEM_HPP__

#include <chrono>
#include <unordered_map>

#include <gz/msgs/pose_v.pb.h>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
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

    /// \brief End of the previously published window for each sensor.
    private: std::unordered_map<gz::sim::Entity,
        std::chrono::steady_clock::duration> lastScanTimes_;

    /// \brief Sensors for which a complete scan window was submitted to
    /// physics during the current update.
    private: std::unordered_map<gz::sim::Entity,
        std::chrono::steady_clock::duration> pendingRaycastEntities_;

    /// \brief Publishes world to sensor-frame transforms for ROS consumers.
    private: gz::transport::Node tfNode_;
    private: gz::transport::Node::Publisher tfPublisher_;
    private: std::chrono::steady_clock::duration lastTfPublishTime_{};
    private: bool tfPublished_{false};

    private: bool IsOwnedBy(
      gz::sim::Entity _sensor,
      gz::sim::Entity _owner,
      const gz::sim::EntityComponentManager &_ecm) const;
  };
}

#endif  //__LIVOX_LIDAR_SYSTEM_HPP__
