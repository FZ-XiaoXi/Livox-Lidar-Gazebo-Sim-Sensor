#include <iostream>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sim/components/CustomSensor.hh>
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
}
 
void LivoxLidarSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
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
        this->entitySensorMap_.emplace(_entity, std::move(sensor));
        gzwarn << "[livox_lidar_gz_sensor] Created Livox sensor type["
               << type << "] name[" << sensorName << "]" << std::endl;
        return true;
      });
}

void LivoxLidarSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  // gzmsg << "LivoxLidarSystem::PostUpdate" << std::endl;
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
