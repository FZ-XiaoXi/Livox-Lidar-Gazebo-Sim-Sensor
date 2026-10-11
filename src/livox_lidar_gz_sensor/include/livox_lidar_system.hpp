/*
 * SPDX-FileCopyrightText: 2026 FZ_XiaoXi
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIVOX_LIDAR_SYSTEM_HPP_
#define LIVOX_LIDAR_SYSTEM_HPP_

#include <gz/msgs/pose_v.pb.h>

#include <chrono>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <string>
#include <unordered_map>

#include "livox_lidar_sensor.hpp"

namespace livox_lidar_system {
class LivoxLidarSystem :
    // This class is a system.
    public gz::sim::System,
    // Configure is called once when Gazebo attaches the plugin to an entity.
    public gz::sim::ISystemConfigure,

    // This class also implements the ISystemPreUpdate interface.
    public gz::sim::ISystemPreUpdate,
    // This class also implements the ISystemPostUpdate interface.
    public gz::sim::ISystemPostUpdate {
 public:
  LivoxLidarSystem();
  ~LivoxLidarSystem() override;
  void Configure(const gz::sim::Entity& _entity,
                 const std::shared_ptr<const sdf::Element>& _element,
                 gz::sim::EntityComponentManager& _ecm,
                 gz::sim::EventManager& _eventManager) override;

  // During PreUpdate, check for new sensors that were inserted into
  // simulation and create more components as needed.
  void PreUpdate(const gz::sim::UpdateInfo& _info,
                 gz::sim::EntityComponentManager& _ecm) override;

  // During PostUpdate, update the known sensors and publish their data. Also
  // remove sensors that have been deleted.
  void PostUpdate(const gz::sim::UpdateInfo& _info,
                  const gz::sim::EntityComponentManager& _ecm) override;

 private:
  /// \brief Remove custom sensors if their entities have been removed from
  /// simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  void RemoveSensorEntities(const gz::sim::EntityComponentManager& _ecm);

  /// \brief A map of custom entities to their sensors
  std::unordered_map<gz::sim::Entity,
                     std::shared_ptr<livox_lidar_sensor::LivoxLidarSensor>>
      entitySensorMap_;

  /// \brief The entity that owns this system.
  gz::sim::Entity ownerEntity_;

  /// \brief End of the previously published window for each sensor.
  std::unordered_map<gz::sim::Entity, std::chrono::steady_clock::duration>
      lastScanTimes_;

  /// \brief Sensors for which a complete scan window was submitted to physics
  /// during the current update.
  std::unordered_map<gz::sim::Entity, std::chrono::steady_clock::duration>
      pendingRaycastEntities_;

  /// \brief Publishes world to sensor-frame transforms for ROS consumers.
  gz::transport::Node tfNode_;
  gz::transport::Node::Publisher tfPublisher_;
  std::string tfTopic_;
  std::chrono::steady_clock::duration lastTfPublishTime_{};
  bool tfPublished_{false};
  bool IsOwnedBy(gz::sim::Entity _sensor, gz::sim::Entity _owner,
                 const gz::sim::EntityComponentManager& _ecm) const;
};
}  // namespace livox_lidar_system

#endif  // LIVOX_LIDAR_SYSTEM_HPP_
