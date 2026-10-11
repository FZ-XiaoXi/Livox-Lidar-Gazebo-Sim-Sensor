/*
 * SPDX-FileCopyrightText: 2026 FZ_XiaoXi
 * SPDX-License-Identifier: Apache-2.0
 */

#include "livox_lidar_system.hpp"

#include <algorithm>
#include <cmath>
#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/Util.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>
#include <sdf/Sensor.hh>

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(livox_lidar_system::LivoxLidarSystem, gz::sim::System,
              livox_lidar_system::LivoxLidarSystem::ISystemConfigure,
              livox_lidar_system::LivoxLidarSystem::ISystemPreUpdate,
              livox_lidar_system::LivoxLidarSystem::ISystemPostUpdate)

using namespace livox_lidar_system;
LivoxLidarSystem::LivoxLidarSystem() {}
LivoxLidarSystem::~LivoxLidarSystem() {}

void LivoxLidarSystem::Configure(
    const gz::sim::Entity& _entity,
    const std::shared_ptr<const sdf::Element>& _sdf,
    gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager&) {
  this->ownerEntity_ = _entity;
  this->tfTopic_ = gz::sim::topicFromScopedName(_entity, _ecm, true);
  if (this->tfTopic_.empty())
    this->tfTopic_ = gz::sim::topicFromScopedName(_entity, _ecm, false);
  if (!this->tfTopic_.empty()) this->tfTopic_ += "/tf";
  if (_sdf && _sdf->HasElement("tf_topic"))
    this->tfTopic_ = _sdf->Get<std::string>("tf_topic");
  this->tfTopic_ = gz::transport::TopicUtils::AsValidTopic(this->tfTopic_);
  if (this->tfTopic_.empty()) {
    gzerr << "[livox_lidar_gz_sensor] Failed to derive a valid TF topic"
          << std::endl;
    return;
  }
  this->tfPublisher_ =
      this->tfNode_.Advertise<gz::msgs::Pose_V>(this->tfTopic_);
  if (!this->tfPublisher_) {
    gzerr << "[livox_lidar_gz_sensor] Failed to advertise " << this->tfTopic_
          << std::endl;
  }
}
void LivoxLidarSystem::PreUpdate(const gz::sim::UpdateInfo& _info,
                                 gz::sim::EntityComponentManager& _ecm) {
  this->pendingRaycastEntities_.clear();
  // gzmsg << "LivoxLidarSystem::PreUpdate" << std::endl;
  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity& _entity,
          const gz::sim::components::CustomSensor* _custom,
          const gz::sim::components::ParentEntity*) -> bool {
        if (!IsOwnedBy(_entity, this->ownerEntity_, _ecm)) return true;

        auto type = gz::sensors::customType(_custom->Data());
        if (type.rfind("livox_", 0) != 0) return true;
        if (this->entitySensorMap_.find(_entity) !=
            this->entitySensorMap_.end()) {
          return true;
        }

        gz::sensors::SensorFactory sensorFactory;
        auto sensor =
            sensorFactory.CreateSensor<livox_lidar_sensor::LivoxLidarSensor>(
                _custom->Data());
        if (nullptr == sensor) {
          gzerr << "Failed to create Livox sensor [" << type << "]"
                << std::endl;
          return true;
        }
        const auto sensorName = sensor->Name();
        const auto scanStart =
            std::max(std::chrono::steady_clock::duration::zero(),
                     _info.simTime - _info.dt);
        this->lastScanTimes_[_entity] = scanStart;
        if (sensor->ScanPeriod() > 0.0) {
          const auto period =
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(sensor->ScanPeriod()));
          sensor->SetNextDataUpdateTime(scanStart + period);
        }
        this->entitySensorMap_.emplace(_entity, std::move(sensor));
        // Custom sensors do not receive these physics-query components from
        // SDF entity creation. Physics needs both components to process a
        // raycast for this entity in the Gazebo 8 API.
        _ecm.CreateComponent(_entity, gz::sim::components::WorldPose(
                                          gz::sim::worldPose(_entity, _ecm)));
        _ecm.CreateComponent(_entity,
                             gz::sim::components::RaycastData(
                                 gz::sim::components::RaycastDataInfo{}));
        gzmsg << "[livox_lidar_gz_sensor] Loaded sensor type[" << type
              << "] name[" << sensorName << "]" << std::endl;
        return true;
      });
  if (_info.paused) return;

  for (auto& [entity, sensor] : this->entitySensorMap_) {
    const auto sensorWorldPose = gz::sim::worldPose(entity, _ecm);
    sensor->SetPose(sensorWorldPose);
    _ecm.SetComponentData<gz::sim::components::WorldPose>(entity,
                                                          sensorWorldPose);

    auto& lastScanTime = this->lastScanTimes_.at(entity);
    if (_info.dt < std::chrono::steady_clock::duration::zero() ||
        _info.simTime < lastScanTime) {
      lastScanTime = _info.simTime;
      const auto period =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(sensor->ScanPeriod()));
      sensor->SetNextDataUpdateTime(lastScanTime + period);
      sensor->SetRaycastResults({});
      _ecm.SetComponentData<gz::sim::components::RaycastData>(entity, {});
      continue;
    }

    // Gazebo 8 raycasts every non-empty RaycastData component on each step.
    // Clear idle requests so a complete frame is traced and consumed once.
    const auto scanEnd = sensor->ScanPeriod() > 0.0
                             ? sensor->NextDataUpdateTime()
                             : _info.simTime;
    if (!sensor->IsActive() || scanEnd > _info.simTime ||
        _info.simTime <= lastScanTime) {
      _ecm.SetComponentData<gz::sim::components::RaycastData>(entity, {});
      continue;
    }

    const double startTime =
        std::chrono::duration<double>(lastScanTime).count();
    const double duration =
        std::chrono::duration<double>(scanEnd - lastScanTime).count();
    auto rays = sensor->GenerateRays(startTime, duration);
    gz::sim::components::RaycastDataInfo raycastData;
    raycastData.rays.reserve(rays.size());
    for (const auto& ray : rays) {
      raycastData.rays.push_back({ray.first, ray.second});
    }
    _ecm.SetComponentData<gz::sim::components::RaycastData>(entity,
                                                            raycastData);
    this->pendingRaycastEntities_[entity] = scanEnd;
  }
}

void LivoxLidarSystem::PostUpdate(const gz::sim::UpdateInfo& _info,
                                  const gz::sim::EntityComponentManager& _ecm) {
  if (_info.paused) return;

  const auto tfPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(0.02));
  const bool publishTf = !this->tfPublished_ ||
                         _info.simTime < this->lastTfPublishTime_ ||
                         _info.simTime - this->lastTfPublishTime_ >= tfPeriod;
  if (publishTf && this->tfPublisher_) {
    for (const auto& [entity, sensor] : this->entitySensorMap_) {
      gz::msgs::Pose_V tfMessage;
      auto* tfPose = tfMessage.add_pose();
      tfPose->set_name(sensor->FrameId());
      gz::msgs::Set(tfPose, gz::sim::worldPose(entity, _ecm));
      auto* header = tfPose->mutable_header();
      *header->mutable_stamp() = gz::msgs::Convert(_info.simTime);
      auto* frame = header->add_data();
      frame->set_key("frame_id");
      frame->add_value("world");
      auto* childFrame = header->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(sensor->FrameId());
      this->tfPublisher_.Publish(tfMessage);
    }
    this->lastTfPublishTime_ = _info.simTime;
    this->tfPublished_ = true;
  }

  for (auto& [entity, sensor] : this->entitySensorMap_) {
    const auto pending = this->pendingRaycastEntities_.find(entity);
    if (pending == this->pendingRaycastEntities_.end()) continue;

    const auto* raycast =
        _ecm.Component<gz::sim::components::RaycastData>(entity);

    if (raycast == nullptr) {
      gzerr << "[livox_lidar_gz_sensor] No RaycastData for sensor entity ["
            << entity << "]" << std::endl;
      continue;
    }

    const auto& data = raycast->Data();

    std::vector<livox_lidar_sensor::LivoxLidarSensor::RaycastResult>
        sensorResults;
    const auto resultCount = std::min(data.results.size(), data.rays.size());
    if (data.results.size() != data.rays.size()) {
      gzwarn << "[livox_lidar_gz_sensor] Raycast result count mismatch for ["
             << sensor->Name() << "]: rays[" << data.rays.size() << "] results["
             << data.results.size() << "]" << std::endl;
    }
    sensorResults.reserve(resultCount);
    for (std::size_t i = 0; i < resultCount; ++i) {
      const auto& result = data.results[i];
      const auto& ray = data.rays[i];

      // RaycastData defines both the hit point and its normalized position on
      // the ray in the sensor entity frame. Use the fraction-derived point
      // when a backend returns a point in another frame or an inconsistent
      // point. This protects the published cloud from isolated backend
      // precision / frame errors without changing valid hit points.
      auto point = result.point;
      if (std::isfinite(result.fraction) && result.fraction >= 0.0 &&
          result.fraction < (1.0 - 1e-6) && ray.start.IsFinite() &&
          ray.end.IsFinite()) {
        const auto rayVector = ray.end - ray.start;
        const auto fractionPoint = ray.start + rayVector * result.fraction;
        const double tolerance = 1e-5 * std::max(1.0, rayVector.Length());
        if (!point.IsFinite() || (point - fractionPoint).Length() > tolerance) {
          point = fractionPoint;
        }
      }
      sensorResults.push_back({point, ray.start, ray.end, result.fraction});
    }
    sensor->SetRaycastResults(std::move(sensorResults));

    // The two-argument overload applies the sensor's SDF update_rate.
    if (sensor->Update(pending->second, false))
      this->lastScanTimes_.at(entity) = pending->second;
  }
  this->pendingRaycastEntities_.clear();
  this->RemoveSensorEntities(_ecm);
}

//////////////////////////////////////////////////
void LivoxLidarSystem::RemoveSensorEntities(
    const gz::sim::EntityComponentManager& _ecm) {
  _ecm.EachRemoved<gz::sim::components::CustomSensor>(
      [&](const gz::sim::Entity& _entity,
          const gz::sim::components::CustomSensor*) -> bool {
        this->entitySensorMap_.erase(_entity);
        this->lastScanTimes_.erase(_entity);
        this->pendingRaycastEntities_.erase(_entity);
        return true;
      });
}

bool LivoxLidarSystem::IsOwnedBy(
    gz::sim::Entity _sensor, gz::sim::Entity _owner,
    const gz::sim::EntityComponentManager& _ecm) const {
  if (_sensor == _owner) return true;

  auto entity = _sensor;

  while (entity != gz::sim::kNullEntity) {
    auto parent = _ecm.Component<gz::sim::components::ParentEntity>(entity);

    if (parent == nullptr) return false;

    entity = parent->Data();

    if (entity == _owner) return true;
  }

  return false;
}
