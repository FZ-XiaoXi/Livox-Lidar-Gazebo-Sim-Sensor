/*
 * SPDX-FileCopyrightText: 2026 FZ_XiaoXi
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIVOX_LIDAR_SENSOR_HPP_
#define LIVOX_LIDAR_SENSOR_HPP_

#include <chrono>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <gz/math/Vector3.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>

namespace livox_lidar_sensor
{
  /// \brief Livox-style custom sensor which publishes packed point clouds.
  class LivoxLidarSensor : public gz::sensors::Sensor
  {
    // Keep the base overload that applies update_rate before dispatching to
    // the one-argument virtual Update implementation.
    public: using gz::sensors::Sensor::Update;

    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return True if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successfull
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;

    /// \brief Check whether the point-cloud topic has subscribers.
    public: virtual bool HasConnections() const override;

    /// \brief Generate rays for [startTime, startTime + duration), in seconds.
    public: std::vector<std::pair<gz::math::Vector3d, gz::math::Vector3d>>
        GenerateRays(double startTime, double duration) const;

    /// \brief Return the scan window corresponding to the sensor update rate.
    /// \return Period in seconds. Zero means update every physics step.
    public: double ScanPeriod() const;

    /// \brief Frame used in the published point cloud.
    public: const std::string &FrameId() const;

    /// \brief Result of one raycast in the sensor frame.
    struct RaycastResult
    {
      gz::math::Vector3d point;
      gz::math::Vector3d rayStart;
      gz::math::Vector3d rayEnd;
      double fraction{std::numeric_limits<double>::infinity()};
    };

    /// \brief Pass physics results to this sensor before Update().
    public: void SetRaycastResults(std::vector<RaycastResult> _results);

    /// \brief Node for communication
    private: gz::transport::Node node_;

    /// \brief Publishes packed point-cloud data
    private: gz::transport::Node::Publisher pointCloudPub_;

    /// \brief Reused packed point-cloud message
    private: gz::msgs::PointCloudPacked pointCloudMsg_;

    private: std::string scanFileName_;
    private: std::string frameId_;
    private: double rangeMin_{0.1};
    private: double rangeMax_{200.0};
    private: unsigned int downsample_{1};

    private: double noiseMean_{0.0};
    private: double noiseStdDev_{0.0};
    private: std::mt19937 noiseGenerator_{0};

    private: std::vector<std::vector<double>> scanRayCsvData_;

    private: double scanPatternDuration_{0.0};

    private: std::vector<RaycastResult> raycastResults_;

    private: std::uint64_t publishCount_{0};

  };
}

#endif  // LIVOX_LIDAR_SENSOR_HPP_
