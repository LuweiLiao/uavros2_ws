/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_IMU_PLUGIN_H

#include <random>
#include <memory>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>

#include <Eigen/Core>

#include "Imu.pb.h"



namespace gazebo {

// Default values for use with ADIS16448 IMU
static constexpr double kDefaultAdisGyroscopeNoiseDensity =
    2.0 * 35.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeRandomWalk =
    2.0 * 4.0 / 3600.0 / 180.0 * M_PI;
static constexpr double kDefaultAdisGyroscopeBiasCorrelationTime =
    1.0e+3;
static constexpr double kDefaultAdisGyroscopeTurnOnBiasSigma =
    0.5 / 180.0 * M_PI;
static constexpr double kDefaultAdisAccelerometerNoiseDensity =
    2.0 * 2.0e-3;
static constexpr double kDefaultAdisAccelerometerRandomWalk =
    2.0 * 3.0e-3;
static constexpr double kDefaultAdisAccelerometerBiasCorrelationTime =
    300.0;
static constexpr double kDefaultAdisAccelerometerTurnOnBiasSigma =
    20.0e-3 * 9.8;
// Earth's gravity in Zurich (lat=+47.3667degN, lon=+8.5500degE, h=+500m, WGS84)
static constexpr double kDefaultGravityMagnitude = 9.8068;

// A description of the parameters:
// https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model-and-Intrinsics
// TODO(burrimi): Should I have a minimalistic description of the params here?
struct ImuParameters {
  /// Gyroscope noise density (two-sided spectrum) [rad/s/sqrt(Hz)]
  double gyroscope_noise_density;
  /// Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
  double gyroscope_random_walk;
  /// Gyroscope bias correlation time constant [s]
  double gyroscope_bias_correlation_time;
  /// Gyroscope turn on bias standard deviation [rad/s]
  double gyroscope_turn_on_bias_sigma;
  /// Accelerometer noise density (two-sided spectrum) [m/s^2/sqrt(Hz)]
  double accelerometer_noise_density;
  /// Accelerometer bias random walk. [m/s^2/s/sqrt(Hz)]
  double accelerometer_random_walk;
  /// Accelerometer bias correlation time constant [s]
  double accelerometer_bias_correlation_time;
  /// Accelerometer turn on bias standard deviation [m/s^2]
  double accelerometer_turn_on_bias_sigma;
  /// Norm of the gravitational acceleration [m/s^2]
  double gravity_magnitude;

  ImuParameters()
      : gyroscope_noise_density(kDefaultAdisGyroscopeNoiseDensity),
        gyroscope_random_walk(kDefaultAdisGyroscopeRandomWalk),
        gyroscope_bias_correlation_time(
            kDefaultAdisGyroscopeBiasCorrelationTime),
        gyroscope_turn_on_bias_sigma(kDefaultAdisGyroscopeTurnOnBiasSigma),
        accelerometer_noise_density(kDefaultAdisAccelerometerNoiseDensity),
        accelerometer_random_walk(kDefaultAdisAccelerometerRandomWalk),
        accelerometer_bias_correlation_time(
            kDefaultAdisAccelerometerBiasCorrelationTime),
        accelerometer_turn_on_bias_sigma(
            kDefaultAdisAccelerometerTurnOnBiasSigma),
        gravity_magnitude(kDefaultGravityMagnitude) {}
};

class GazeboImuPlugin : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override;
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override;
  void PostUpdate(const gz::sim::UpdateInfo &, const gz::sim::EntityComponentManager &) override;
 protected:
  void AddNoise(Eigen::Vector3d *, Eigen::Vector3d *, double);
 private:
  gz::sim::Entity model_{gz::sim::kNullEntity}, link_{gz::sim::kNullEntity};
  std::string namespace_, imu_topic_, link_name_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher imu_pub_, connect_pub_;
  gz_sensor_msgs::Imu imu_message_;
  gz::math::Vector3d gravity_W_;
  ImuParameters imu_parameters_;
  std::default_random_engine random_generator_;
  std::normal_distribution<double> standard_normal_distribution_{0.0, 1.0};
  Eigen::Vector3d gyroscope_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyroscope_turn_on_bias_, accelerometer_turn_on_bias_;
  double last_route_time_{-1.0};
};
}  // namespace gazebo
#endif
