#ifndef __LIVOX_LIDAR_SENSOR_HPP__
#define __LIVOX_LIDAR_SENSOR_HPP__

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/transport/Node.hh>
#include "livox_lidar_profile.hpp"

namespace livox_lidar_sensor
{
  /// \brief Example sensor that publishes the total distance travelled by a
  /// robot, with noise.
  class LivoxLidarSensor : public gz::sensors::Sensor
  {
    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successfull
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Set the current postiion of the robot, so the odometer can
    /// calculate the distance travelled.
    /// \param[in] _pos Current position in world coordinates.
    public: void NewPosition(const gz::math::Vector3d &_pos);

    /// \brief Get the latest world postiion of the robot.
    /// \return The latest position given to the odometer.
    public: const gz::math::Vector3d &Position() const;

    /// \brief Previous position of the robot.
    private: gz::math::Vector3d prevPos{std::nan(""), std::nan(""),
        std::nan("")};

    /// \brief Latest total distance.
    private: double totalDistance{0.0};

    /// \brief Noise that will be applied to the sensor data
    private: gz::sensors::NoisePtr noise{nullptr};

    /// \brief Node for communication
    private: gz::transport::Node node;

    /// \brief Publishes sensor data
    private: gz::transport::Node::Publisher pub;

    private: std::string model_;
    private: std::string scanFileName_;
    private: std::string frameId_;
    private: double rangeMin_{0.1};
    private: double rangeMax_{200.0};
    private: unsigned int downsample_{1};
  };
}

#endif  //__LIVOX_LIDAR_SENSOR_HPP__
