#include <math.h>

#include <gz/msgs/double.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include "livox_lidar_sensor.hpp"
#include "csv_reader.hpp"

using namespace livox_lidar_sensor;

//////////////////////////////////////////////////
bool LivoxLidarSensor::Load(const sdf::Sensor &_sdf)
{
	const auto type = gz::sensors::customType(_sdf);
	// const auto *profile = LivoxProfileRegistry::Find(type);
	// if (nullptr == profile){
	// 	gzerr << "[livox_lidar_gz_sensor] Failed to find Livox profile: " << type << std::endl;
	// 	return false;
	// }
	// // this->profile_ = *profile;

	// Load common sensor params
	if (!gz::sensors::Sensor::Load(_sdf)){
        gzerr << "[livox_lidar_gz_sensor] Failed to load sensor: " << this->Name() << std::endl;
		return false;
	}

    gzwarn << "[livox_lidar_gz_sensor] Loading Livox LIDAR type[" << type << "]" << " name[" << this->Name() << "]" << " update rate[" << this->UpdateRate() << "]" << std::endl;
	auto element = _sdf.Element();
	if (element == nullptr || !element->HasElement("gz:livox_config"))
	{
		gzerr << "Missing <gz:livox_config> configuration for [" << _sdf.Name() << "]" << std::endl;
		return false;
	}
	auto livox_config = element->FindElement("gz:livox_config");
	if (livox_config == nullptr)
	{
		gzerr << "Failed to find <gz:livox_config> for [" << _sdf.Name() << "]" << std::endl;
		return false;
	}


	if (livox_config->HasElement("model"))
    	this->model_ = livox_config->Get<std::string>("model");

	if (livox_config->HasElement("scan_file"))
		this->scanFileName_ = livox_config->Get<std::string>("scan_file");

	if (livox_config->HasElement("frame_id"))
		this->frameId_ = livox_config->Get<std::string>("frame_id");
	else
		this->frameId_ = _sdf.Name();

	if (livox_config->HasElement("range_min"))
		this->rangeMin_ = livox_config->Get<double>("range_min");

	if (livox_config->HasElement("range_max"))
		this->rangeMax_ = livox_config->Get<double>("range_max");

	if (livox_config->HasElement("downsample"))
		this->downsample_ = livox_config->Get<unsigned int>("downsample");

	std::vector<std::vector<double>> datas;
	if(scanFileName_.empty()){
		gzerr << "[livox_lidar_gz_sensor] Missing scan_file for [" << _sdf.Name() << "]" << std::endl;
		return false;
	}
	if (!CsvReader::ReadCsvFile(scanFileName_, datas)) {
        gzerr << "[livox_lidar_gz_sensor] Failed to read CSV file: " << scanFileName_ << std::endl;
        return false;
    }
	
	gzwarn << "[livox_lidar_gz_sensor] Livox LIDAR Initialized Successfully - type[" << type << "]" << " name[" << this->Name() << "]" << " model[" << this->model_ << "]" << " scan_file[" << this->scanFileName_ << "]" << " frame_id[" << this->frameId_ << "]" << " range_min[" << this->rangeMin_ << "]" << " range_max[" << this->rangeMax_ << "]" << " downsample[" << this->downsample_ << "]"  << std::endl;
	
	

	
	

	// // Advertise topic where data will be published
	// this->pub = this->node.Advertise<gz::msgs::Double>(this->Topic());

	// if (!_sdf.Element()->HasElement("gz:odometer"))
	// {
	// 	gzdbg << "No custom configuration for [" << this->Topic() << "]"
	// 				 << std::endl;
	// 	return true;
	// }

	// // Load custom sensor params
	// auto customElem = _sdf.Element()->FindElement("gz:odometer");

	// if (!customElem->HasElement("noise"))
	// {
	// 	gzdbg << "No noise for [" << this->Topic() << "]" << std::endl;
	// 	return true;
	// }

	// sdf::Noise noiseSdf;
	// noiseSdf.Load(customElem->FindElement("noise"));
	// this->noise = gz::sensors::NoiseFactory::NewNoiseModel(noiseSdf);
	// if (nullptr == this->noise)
	// {
	// 	gzerr << "Failed to load noise." << std::endl;
	// 	return false;
	// }

	return true;
}

//////////////////////////////////////////////////
bool LivoxLidarSensor::Update(const std::chrono::steady_clock::duration &_now)
{
	gzerr << "LivoxLidarSensor::Update" << std::endl;
	// gz::msgs::Double msg;
	// *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
	// auto frame = msg.mutable_header()->add_data();
	// frame->set_key("frame_id");
	// frame->add_value(this->Name());

	// this->totalDistance = this->noise->Apply(this->totalDistance);

	// msg.set_data(this->totalDistance);

	// this->AddSequence(msg.mutable_header());
	// this->pub.Publish(msg);

	return true;
}

//////////////////////////////////////////////////
void LivoxLidarSensor::NewPosition(const gz::math::Vector3d &_pos)
{
	if (!isnan(this->prevPos.X()))
	{
		this->totalDistance += this->prevPos.Distance(_pos);
	}
	this->prevPos = _pos;
}

//////////////////////////////////////////////////
const gz::math::Vector3d &LivoxLidarSensor::Position() const
{
	return this->prevPos;
}
