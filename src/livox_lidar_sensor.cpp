#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>

#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/pointcloud_packed.pb.h>

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

	if (livox_config->HasElement("noise"))
	{
		const auto noise = livox_config->FindElement("noise");
		if (noise->HasElement("mean"))
			this->noiseMean_ = noise->Get<double>("mean");
		if (noise->HasElement("stddev"))
			this->noiseStdDev_ = noise->Get<double>("stddev");
	}
	if (livox_config->HasElement("noise_mean"))
		this->noiseMean_ = livox_config->Get<double>("noise_mean");
	if (livox_config->HasElement("noise_stddev"))
		this->noiseStdDev_ = livox_config->Get<double>("noise_stddev");
	if (!std::isfinite(this->noiseMean_) ||
		!std::isfinite(this->noiseStdDev_) || this->noiseStdDev_ < 0.0)
	{
		gzerr << "[livox_lidar_gz_sensor] noise mean must be finite and "
		      << "stddev must be finite and non-negative" << std::endl;
		return false;
	}

	std::vector<std::vector<double>> datas;
	if(scanFileName_.empty()){
		gzerr << "[livox_lidar_gz_sensor] Missing scan_file for [" << _sdf.Name() << "]" << std::endl;
		return false;
	}
	if (!CsvReader::ReadCsvFile(scanFileName_, datas)) {
        gzerr << "[livox_lidar_gz_sensor] Failed to read CSV file: " << scanFileName_ << std::endl;
        return false;
    }
	if (this->downsample_ == 0)
	{
		gzerr << "[livox_lidar_gz_sensor] downsample must be at least 1"
		      << std::endl;
		return false;
	}
	if (datas.size() < 2)
	{
		gzerr << "[livox_lidar_gz_sensor] CSV needs at least two timed samples"
		      << std::endl;
		return false;
	}
	for (std::size_t i = 0; i < datas.size(); ++i)
	{
		const auto &sample = datas[i];
		if (sample.size() != 3 || !std::isfinite(sample[0]) ||
			!std::isfinite(sample[1]) || !std::isfinite(sample[2]) ||
			(i > 0 && sample[0] <= datas[i - 1][0]))
		{
			gzerr << "[livox_lidar_gz_sensor] CSV samples must have three "
			      << "finite values and strictly increasing timestamps"
			      << std::endl;
			return false;
		}
	}

	// Normalize the pattern to time zero. Preserve its original duration
	// before decimation so skipping rows does not slow down the scan.
	const double firstSampleTime = datas.front()[0];
	const double sampleInterval =
		datas.back()[0] - datas[datas.size() - 2][0];
	this->scanPatternDuration_ =
		datas.back()[0] - firstSampleTime + sampleInterval;
	this->scanRayCsvData_.clear();
	this->scanRayCsvData_.reserve(
		(datas.size() - 1) / this->downsample_ + 1);
	for (std::size_t i = 0; i < datas.size(); i += this->downsample_)
	{
		datas[i][0] -= firstSampleTime;
		this->scanRayCsvData_.push_back(std::move(datas[i]));
	}

	// The SDF topic is the public Gazebo transport topic. Keep a useful
	// per-sensor fallback when a caller omits <topic>.
	if (this->Topic().empty())
		this->SetTopic("/" + this->Name() + "/points");

	gz::msgs::InitPointCloudPacked(
		this->pointCloudMsg_, this->frameId_, false,
		{{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32},
		 {"intensity", gz::msgs::PointCloudPacked::Field::FLOAT32}});
	this->pointCloudPub_ =
		this->node_.Advertise<gz::msgs::PointCloudPacked>(this->Topic());
	if (!this->pointCloudPub_)
	{
		gzerr << "[livox_lidar_gz_sensor] Unable to advertise point cloud topic ["
		      << this->Topic() << "]" << std::endl;
		return false;
	}
	
	gzerr << "[livox_lidar_gz_sensor] Livox LIDAR Initialized Successfully - type[" << type << "]" << " name[" << this->Name() << "]" << " model[" << this->model_ << "]" << " scan_file[" << this->scanFileName_ << "]" << " frame_id[" << this->frameId_ << "]" << " range_min[" << this->rangeMin_ << "]" << " range_max[" << this->rangeMax_ << "]" << " downsample[" << this->downsample_ << "]" << " csv_points[" << this->scanRayCsvData_.size() << "/" << datas.size() << "]" << " pattern_period[" << this->scanPatternDuration_ << "]" << " topic[" << this->Topic() << "]" << std::endl;
	
	

	
	

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
	if (!this->pointCloudPub_)
		return false;

	std::vector<gz::math::Vector3d> outputPoints;
	outputPoints.reserve(this->raycastResults_.size());
	for (const auto &result : this->raycastResults_)
	{
		if (std::isfinite(result.fraction) && result.fraction >= 0.0 &&
			result.fraction < 1.0 && result.point.IsFinite())
		{
			gz::math::Vector3d outputPoint = result.point;
			if (this->noiseStdDev_ > 0.0 || this->noiseMean_ != 0.0)
			{
				const auto ray = result.rayEnd - result.rayStart;
				const double rayLength = ray.Length();
				if (!(rayLength > 0.0) || !std::isfinite(rayLength))
					continue;
				const auto direction = ray / rayLength;
				const double range = (result.point - result.rayStart).Dot(direction);
				std::normal_distribution<double> distribution(
					this->noiseMean_, this->noiseStdDev_);
				const double noisyRange = range + distribution(this->noiseGenerator_);
				if (!std::isfinite(noisyRange) || noisyRange < this->rangeMin_ ||
					noisyRange > this->rangeMax_)
					continue;
				outputPoint = result.rayStart + direction * noisyRange;
			}
			outputPoints.push_back(outputPoint);
		}
	}
	const std::size_t pointCount = outputPoints.size();

	this->pointCloudMsg_.set_width(static_cast<uint32_t>(pointCount));
	this->pointCloudMsg_.set_height(1);
	this->pointCloudMsg_.set_row_step(
		static_cast<uint32_t>(this->pointCloudMsg_.point_step() * pointCount));
	this->pointCloudMsg_.set_is_dense(true);
	this->pointCloudMsg_.mutable_data()->assign(
		this->pointCloudMsg_.row_step(), '\0');

	*this->pointCloudMsg_.mutable_header()->mutable_stamp() =
		gz::msgs::Convert(_now);
	this->pointCloudMsg_.mutable_header()->clear_data();
	auto *frame = this->pointCloudMsg_.mutable_header()->add_data();
	frame->set_key("frame_id");
	frame->add_value(this->frameId_);

	if (pointCount > 0)
	{
		gz::msgs::PointCloudPackedIterator<float> xIter(
			this->pointCloudMsg_, "x");
		gz::msgs::PointCloudPackedIterator<float> yIter(
			this->pointCloudMsg_, "y");
		gz::msgs::PointCloudPackedIterator<float> zIter(
			this->pointCloudMsg_, "z");
		gz::msgs::PointCloudPackedIterator<float> intensityIter(
			this->pointCloudMsg_, "intensity");

		for (const auto &outputPoint : outputPoints)
		{
			*xIter = static_cast<float>(outputPoint.X());
			*yIter = static_cast<float>(outputPoint.Y());
			*zIter = static_cast<float>(outputPoint.Z());
			*intensityIter = 1.0F;
			++xIter;
			++yIter;
			++zIter;
			++intensityIter;
		}
	}

	this->AddSequence(this->pointCloudMsg_.mutable_header());
	this->pointCloudPub_.Publish(this->pointCloudMsg_);
	if ((++this->publishCount_ % 10) == 0)
	{
		gzmsg << "[livox_lidar_gz_sensor] published cloud sensor["
		      << this->Name() << "] topic[" << this->Topic() << "] rays["
		      << this->raycastResults_.size() << "] points["
		      << pointCount << "] connections["
		      << this->pointCloudPub_.HasConnections() << "]" << std::endl;
	}
	this->raycastResults_.clear();

	return true;
}

//////////////////////////////////////////////////
bool LivoxLidarSensor::HasConnections() const
{
	return this->pointCloudPub_ && this->pointCloudPub_.HasConnections();
}

//////////////////////////////////////////////////
void LivoxLidarSensor::SetRaycastResults(
	std::vector<RaycastResult> _results)
{
	this->raycastResults_ = std::move(_results);
}

//////////////////////////////////////////////////
void LivoxLidarSensor::NewPosition(const gz::math::Vector3d &_pos)
{
	if (!std::isnan(this->prevPos.X()))
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

double LivoxLidarSensor::ScanPeriod() const
{
	return this->UpdateRate() > 0.0 ? 1.0 / this->UpdateRate() : 0.0;
}

//////////////////////////////////////////////////
const std::string &LivoxLidarSensor::FrameId() const
{
	return this->frameId_;
}

std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
LivoxLidarSensor::GenerateRays(double startTime, double duration) const
{
	std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>> rays;
	if (this->scanRayCsvData_.empty() || !std::isfinite(startTime) ||
		!std::isfinite(duration) || !(duration > 0.0))
		return rays;

	constexpr double timeTolerance = 1e-9;
	double windowStart = std::fmod(
		std::max(0.0, startTime), this->scanPatternDuration_);
	if (this->scanPatternDuration_ - windowStart < timeTolerance)
		windowStart = 0.0;

	const double degreesToRadians = M_PI / 180.0;
	const double startRange = this->rangeMin_;
	const double endRange = this->rangeMax_;
	const auto addSample = [&](std::size_t index) {
		const auto &sample = this->scanRayCsvData_[index];

		// The source files use azimuth and zenith in degrees. The legacy Livox
		// model converts zenith=90 degrees to the sensor's horizontal plane.
		const double azimuth = sample[1] * degreesToRadians;
		const double elevation = sample[2] * degreesToRadians - M_PI_2;
		const gz::math::Quaterniond rotation(0.0, elevation, azimuth);
		const auto direction = rotation.RotateVector(gz::math::Vector3d::UnitX);
		rays.emplace_back(direction * startRange, direction * endRange);
	};

	const auto firstAfter = [&](double sampleTime) {
		return static_cast<std::size_t>(std::lower_bound(
			this->scanRayCsvData_.begin(), this->scanRayCsvData_.end(),
			sampleTime + timeTolerance,
			[](const std::vector<double> &_sample, double _time) {
				return _sample[0] < _time;
			}) - this->scanRayCsvData_.begin());
	};
	const auto afterLast = [&](double sampleTime) {
		return static_cast<std::size_t>(std::upper_bound(
			this->scanRayCsvData_.begin(), this->scanRayCsvData_.end(),
			sampleTime + timeTolerance,
			[](double _time, const std::vector<double> &_sample) {
				return _time < _sample[0];
			}) - this->scanRayCsvData_.begin());
	};
	const auto processRange = [&](std::size_t begin, std::size_t end) {
		for (std::size_t index = begin; index < end; ++index)
			addSample(index);
	};

	// Split windows across the original CSV duration, including windows longer
	// than one full pattern. Right-closed windows include the endpoint sample
	// and exclude the start sample, so adjacent frames neither lose nor repeat
	// CSV rows.
	double remaining = duration;
	while (remaining > timeTolerance)
	{
		const double segment = std::min(
			remaining, this->scanPatternDuration_ - windowStart);
		const double windowEnd = windowStart + segment;
		processRange(firstAfter(windowStart), afterLast(windowEnd));
		remaining -= segment;
		windowStart = 0.0;
	}
	return rays;
}
