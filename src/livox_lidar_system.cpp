#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Util.hh>
#include <sdf/Sensor.hh>
#include <gz/sensors/Util.hh>
#include "livox_lidar_system.hpp"

 
// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    livox_lidar_system::LivoxLidarSystem,
    gz::sim::System,
    livox_lidar_system::LivoxLidarSystem::ISystemConfigure,
    livox_lidar_system::LivoxLidarSystem::ISystemPreUpdate,
    livox_lidar_system::LivoxLidarSystem::ISystemPostUpdate)

using namespace livox_lidar_system;
 
LivoxLidarSystem::LivoxLidarSystem()
{

}
 
LivoxLidarSystem::~LivoxLidarSystem()
{
}

void LivoxLidarSystem::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &)
{
  // std::cerr is independent of Gazebo's verbosity filtering.
  gzmsg << "#################################################################" << std::endl;
  gzmsg << "#[livox_lidar_gz_sensor] LivoxLidarSystem started on entity " << _entity << std::endl;
  gzmsg << "#################################################################" << std::endl;
  this->ownerEntity_ = _entity;
  this->tfPublisher_ = this->tfNode_.Advertise<gz::msgs::Pose_V>(
      "/livox/tf");
  if (!this->tfPublisher_)
  {
    gzerr << "[livox_lidar_gz_sensor] Failed to advertise /livox/tf"
           << std::endl;
  }
}
 
void LivoxLidarSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  this->pendingRaycastEntities_.clear();
  // gzmsg << "LivoxLidarSystem::PreUpdate" << std::endl;
  _ecm.EachNew<gz::sim::components::CustomSensor,
               gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_custom,
        const gz::sim::components::ParentEntity *_parent)->bool
      {
        if (!IsOwnedBy(_entity, this->ownerEntity_, _ecm))
          return true;

        auto type = gz::sensors::customType(_custom->Data());
        if (type.rfind("livox_", 0) != 0)
          return true;
        gzwarn << "[livox_lidar_gz_sensor] Found new Livox sensor [" << type << "]" << std::endl;
        if (this->entitySensorMap_.find(_entity) != this->entitySensorMap_.end()){
          return true;
        }
          
        gz::sensors::SensorFactory sensorFactory;
        auto sensor = sensorFactory.CreateSensor<livox_lidar_sensor::LivoxLidarSensor>(_custom->Data());
        if (nullptr == sensor)
        {
          gzerr << "Failed to create Livox sensor [" << type << "]" << std::endl;
          return true;
        }
        const auto sensorName = sensor->Name();
        const auto scanStart = std::max(
            std::chrono::steady_clock::duration::zero(),
            _info.simTime - _info.dt);
        this->lastScanTimes_[_entity] = scanStart;
        if (sensor->ScanPeriod() > 0.0)
        {
          const auto period = std::chrono::duration_cast<
              std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(sensor->ScanPeriod()));
          sensor->SetNextDataUpdateTime(scanStart + period);
        }
        this->entitySensorMap_.emplace(_entity, std::move(sensor));
        // Custom sensors do not receive these physics-query components from
        // SDF entity creation. Physics needs both components to process a
        // raycast for this entity in the Gazebo 8 API.
        _ecm.CreateComponent(_entity,
            gz::sim::components::WorldPose(gz::sim::worldPose(_entity, _ecm)));
        _ecm.CreateComponent(_entity,
            gz::sim::components::RaycastData(
                gz::sim::components::RaycastDataInfo{}));
        gzwarn << "[livox_lidar_gz_sensor] Created Livox sensor type["
               << type << "] name[" << sensorName << "]" << std::endl;
        return true;
      });
  if(_info.paused)
    return;

  for (auto &[entity, sensor] : this->entitySensorMap_)
  {
    _ecm.SetComponentData<gz::sim::components::WorldPose>(
        entity, gz::sim::worldPose(entity, _ecm));

    auto &lastScanTime = this->lastScanTimes_.at(entity);
    if (_info.dt < std::chrono::steady_clock::duration::zero() ||
        _info.simTime < lastScanTime)
    {
      lastScanTime = _info.simTime;
      const auto period = std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(sensor->ScanPeriod()));
      sensor->SetNextDataUpdateTime(lastScanTime + period);
      sensor->SetRaycastResults({});
      _ecm.SetComponentData<gz::sim::components::RaycastData>(entity, {});
      continue;
    }

    // Gazebo 8 raycasts every non-empty RaycastData component on each step.
    // Clear idle requests so a complete frame is traced and consumed once.
    const auto scanEnd = sensor->ScanPeriod() > 0.0
        ? sensor->NextDataUpdateTime() : _info.simTime;
    if (!sensor->IsActive() ||
        scanEnd > _info.simTime ||
        _info.simTime <= lastScanTime)
    {
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
    for (const auto &ray : rays)
    {
      raycastData.rays.push_back({ray.first, ray.second});
    }
    _ecm.SetComponentData<gz::sim::components::RaycastData>(
        entity, raycastData);
    this->pendingRaycastEntities_[entity] = scanEnd;
  }
}

void LivoxLidarSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  const auto tfPeriod = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(std::chrono::duration<double>(0.02));
  const bool publishTf = !this->tfPublished_ ||
      _info.simTime < this->lastTfPublishTime_ ||
      _info.simTime - this->lastTfPublishTime_ >= tfPeriod;
  if (publishTf && this->tfPublisher_)
  {
    for (const auto &[entity, sensor] : this->entitySensorMap_)
    {
      gz::msgs::Pose_V tfMessage;
      auto *tfPose = tfMessage.add_pose();
      tfPose->set_name(sensor->FrameId());
      gz::msgs::Set(tfPose, gz::sim::worldPose(entity, _ecm));
      auto *header = tfPose->mutable_header();
      *header->mutable_stamp() = gz::msgs::Convert(_info.simTime);
      auto *frame = header->add_data();
      frame->set_key("frame_id");
      frame->add_value("world");
      auto *childFrame = header->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(sensor->FrameId());
      this->tfPublisher_.Publish(tfMessage);
    }
    this->lastTfPublishTime_ = _info.simTime;
    this->tfPublished_ = true;
  }

  for (auto &[entity, sensor] : this->entitySensorMap_)
  {
    const auto pending = this->pendingRaycastEntities_.find(entity);
    if (pending == this->pendingRaycastEntities_.end())
      continue;

    const auto *raycast =
        _ecm.Component<gz::sim::components::RaycastData>(entity);

    if (raycast == nullptr)
    {
      gzerr << "[livox_lidar_gz_sensor] No RaycastData for sensor entity ["
            << entity << "]" << std::endl;
      continue;
    }

    const auto &data = raycast->Data();
    std::size_t hitCount = 0;
    for (std::size_t i = 0; i < data.results.size(); ++i)
    {
      const auto &result = data.results[i];
      if (std::isfinite(result.fraction) &&
          result.fraction >= 0.0 && result.fraction < 1.0 &&
          result.point.IsFinite())
      {
        ++hitCount;
      }
    }

    // Log the complete scan window, rather than the last 1 ms physics batch.
    {
      const double scanEnd =
          std::chrono::duration<double>(pending->second).count();
      gzerr << "[livox_lidar_gz_sensor] time["
            << std::fixed << std::setprecision(5) << scanEnd
            << "] sensor[" << sensor->Name() << "] rays["
            << data.rays.size() << "] results[" << data.results.size()
            << "] hits[" << hitCount << "] window["
            << std::chrono::duration<double>(
                this->lastScanTimes_.at(entity)).count()
            << ", " << scanEnd << ")" << std::endl;

      // const auto count = std::min<std::size_t>(4, data.results.size());
      // for (std::size_t i = 0; i < count; ++i)
      // {
      //   // const auto &ray = i < data.rays.size() ? data.rays[i]
      //   //                                        : gz::sim::components::RayInfo{};
      //   // const auto &result = data.results[i];
      //   // gzerr << "  ray[" << i << "] start["
      //   //       << ray.start.X() << ", " << ray.start.Y() << ", "
      //   //       << ray.start.Z() << "] end[" << ray.end.X() << ", "
      //   //       << ray.end.Y() << ", " << ray.end.Z() << "] fraction["
      //   //       << result.fraction << "] point[" << result.point.X() << ", "
      //   //       << result.point.Y() << ", " << result.point.Z() << "]"
      //   //       << std::endl;
      // }
    }

    std::vector<livox_lidar_sensor::LivoxLidarSensor::RaycastResult>
        sensorResults;
    sensorResults.reserve(data.results.size());
    for (std::size_t i = 0; i < data.results.size(); ++i)
    {
      const auto &result = data.results[i];
      const auto ray = i < data.rays.size() ? data.rays[i] :
          gz::sim::components::RayInfo{};
      sensorResults.push_back({result.point, ray.start, ray.end,
          result.fraction, result.normal});
    }
    sensor->SetRaycastResults(std::move(sensorResults));

    // The two-argument overload applies the sensor's SDF update_rate.
    if (sensor->Update(pending->second, false))
      this->lastScanTimes_.at(entity) = pending->second;
  }
  this->pendingRaycastEntities_.clear();
}

bool LivoxLidarSystem::IsOwnedBy(
  gz::sim::Entity _sensor,
  gz::sim::Entity _owner,
  const gz::sim::EntityComponentManager &_ecm) const
{
  if (_sensor == _owner)
    return true;

  auto entity = _sensor;

  while (entity != gz::sim::kNullEntity)
  {
    auto parent =
        _ecm.Component<gz::sim::components::ParentEntity>(entity);

    if (parent == nullptr)
      return false;

    entity = parent->Data();

    if (entity == _owner)
      return true;
  }

  return false;
}
