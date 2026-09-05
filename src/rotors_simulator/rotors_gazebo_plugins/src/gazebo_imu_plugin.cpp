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

// Gazebo Sim API port; preserve the original RotorS IMU noise equations,
// frame, protobuf schema, topic and bridge registration responsibility.
#include "rotors_gazebo_plugins/gazebo_imu_plugin.h"
#include "ConnectGazeboToRosTopic.pb.h"
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <cassert>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>

namespace gazebo {
void GazeboImuPlugin::Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &, gz::sim::EventManager &) {
  model_ = entity;
  namespace_ = sdf->Get("robotNamespace", std::string("")).first;
  link_name_ = sdf->Get("linkName", std::string("")).first;
  imu_topic_ = sdf->Get("imuTopic", std::string("imu")).first;
  imu_parameters_.gyroscope_noise_density = sdf->Get("gyroscopeNoiseDensity", imu_parameters_.gyroscope_noise_density).first;
  imu_parameters_.gyroscope_random_walk = sdf->Get("gyroscopeBiasRandomWalk", imu_parameters_.gyroscope_random_walk).first;
  imu_parameters_.gyroscope_bias_correlation_time = sdf->Get("gyroscopeBiasCorrelationTime", imu_parameters_.gyroscope_bias_correlation_time).first;
  imu_parameters_.gyroscope_turn_on_bias_sigma = sdf->Get("gyroscopeTurnOnBiasSigma", imu_parameters_.gyroscope_turn_on_bias_sigma).first;
  imu_parameters_.accelerometer_noise_density = sdf->Get("accelerometerNoiseDensity", imu_parameters_.accelerometer_noise_density).first;
  imu_parameters_.accelerometer_random_walk = sdf->Get("accelerometerRandomWalk", imu_parameters_.accelerometer_random_walk).first;
  imu_parameters_.accelerometer_bias_correlation_time = sdf->Get("accelerometerBiasCorrelationTime", imu_parameters_.accelerometer_bias_correlation_time).first;
  imu_parameters_.accelerometer_turn_on_bias_sigma = sdf->Get("accelerometerTurnOnBiasSigma", imu_parameters_.accelerometer_turn_on_bias_sigma).first;
  if (imu_parameters_.gyroscope_bias_correlation_time <= 0 ||
      imu_parameters_.accelerometer_bias_correlation_time <= 0)
    throw std::runtime_error("RotorS IMU bias correlation time must be positive");
  imu_message_.mutable_header()->set_frame_id(link_name_);
  for (int i = 0; i < 9; ++i) {
    imu_message_.add_orientation_covariance(-1.0);
    imu_message_.add_angular_velocity_covariance(i % 4 == 0 ?
        std::pow(imu_parameters_.gyroscope_noise_density, 2) : 0.0);
    imu_message_.add_linear_acceleration_covariance(i % 4 == 0 ?
        std::pow(imu_parameters_.accelerometer_noise_density, 2) : 0.0);
  }
  for (int i = 0; i < 3; ++i) {
    gyroscope_turn_on_bias_[i] = imu_parameters_.gyroscope_turn_on_bias_sigma *
        standard_normal_distribution_(random_generator_);
    accelerometer_turn_on_bias_[i] = imu_parameters_.accelerometer_turn_on_bias_sigma *
        standard_normal_distribution_(random_generator_);
  }
}

void GazeboImuPlugin::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &ecm) {
  if (link_ != gz::sim::kNullEntity) return;
  auto matches = gz::sim::entitiesFromScopedName(link_name_, ecm, model_);
  for (auto entity : matches)
    if (ecm.EntityHasComponentType(entity, gz::sim::components::Link::typeId)) {
      link_ = entity;
      break;
    }
  if (link_ == gz::sim::kNullEntity) return;
  gz::sim::Link(link_).EnableVelocityChecks(ecm);
  gz::sim::Link(link_).EnableAccelerationChecks(ecm);
  const auto world = gz::sim::worldEntity(model_, ecm);
  gravity_W_ = ecm.Component<gz::sim::components::Gravity>(world)->Data();
  imu_parameters_.gravity_magnitude = gravity_W_.Length();
  const std::string prefix = "/gazebo/" + ecm.Component<gz::sim::components::Name>(world)->Data();
  imu_pub_ = node_.Advertise<gz_sensor_msgs::Imu>(prefix + "/" + namespace_ + "/" + imu_topic_);
  connect_pub_ = node_.Advertise<gz_std_msgs::ConnectGazeboToRosTopic>(
      prefix + "/connect_gazebo_to_ros_subtopic");
  gzmsg << "[GazeboImuPlugin] resolved " << link_name_ << "; ROS topic /"
        << namespace_ << "/" << imu_topic_ << "\n";
}

void GazeboImuPlugin::AddNoise(Eigen::Vector3d* linear_acceleration,
                               Eigen::Vector3d* angular_velocity,
                               const double dt) {
  assert(linear_acceleration != nullptr);
  assert(angular_velocity != nullptr);
  assert(dt > 0.0);

  // Gyrosocpe
  double tau_g = imu_parameters_.gyroscope_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * imu_parameters_.gyroscope_noise_density;
  double sigma_b_g = imu_parameters_.gyroscope_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2.0 *
                            (exp(-2.0 * dt / tau_g) - 1.0));
  // Compute state-transition.
  double phi_g_d = exp(-1.0 / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i) {
    gyroscope_bias_[i] =
        phi_g_d * gyroscope_bias_[i] +
        sigma_b_g_d * standard_normal_distribution_(random_generator_);
    (*angular_velocity)[i] =
        (*angular_velocity)[i] + gyroscope_bias_[i] +
        sigma_g_d * standard_normal_distribution_(random_generator_) +
        gyroscope_turn_on_bias_[i];
  }

  // Accelerometer
  double tau_a = imu_parameters_.accelerometer_bias_correlation_time;
  // Discrete-time standard deviation equivalent to an "integrating" sampler
  // with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * imu_parameters_.accelerometer_noise_density;
  double sigma_b_a = imu_parameters_.accelerometer_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2.0 *
                            (exp(-2.0 * dt / tau_a) - 1.0));
  // Compute state-transition.
  double phi_a_d = exp(-1.0 / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear
  // acceleration.
  for (int i = 0; i < 3; ++i) {
    accelerometer_bias_[i] =
        phi_a_d * accelerometer_bias_[i] +
        sigma_b_a_d * standard_normal_distribution_(random_generator_);
    (*linear_acceleration)[i] =
        (*linear_acceleration)[i] + accelerometer_bias_[i] +
        sigma_a_d * standard_normal_distribution_(random_generator_) +
        accelerometer_turn_on_bias_[i];
  }
}


void GazeboImuPlugin::PostUpdate(const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm) {
  if (info.paused || link_ == gz::sim::kNullEntity) return;
  const double dt = std::chrono::duration<double>(info.dt).count();
  if (dt <= 0.0) return;
  const double now = std::chrono::duration<double>(info.simTime).count();
  if ((!imu_pub_.HasConnections() || last_route_time_ < 0.0) &&
      now - last_route_time_ >= 1.0 && connect_pub_.HasConnections()) {
    gz_std_msgs::ConnectGazeboToRosTopic request;
    request.set_gazebo_topic("~/" + namespace_ + "/" + imu_topic_);
    request.set_ros_topic(namespace_ + "/" + imu_topic_);
    request.set_msgtype(gz_std_msgs::ConnectGazeboToRosTopic::IMU);
    connect_pub_.Publish(request);
    last_route_time_ = now;
  }
  const gz::sim::Link link(link_);
  const auto pose = link.WorldPose(ecm);
  const auto accel = link.WorldLinearAcceleration(ecm);
  const auto angular = link.WorldAngularVelocity(ecm);
  if (!pose || !accel || !angular) return;
  const auto rotation = pose->Rot();
  const auto acceleration_I = rotation.RotateVectorReverse(*accel - gravity_W_);
  const auto angular_I = rotation.RotateVectorReverse(*angular);
  Eigen::Vector3d a(acceleration_I.X(), acceleration_I.Y(), acceleration_I.Z());
  Eigen::Vector3d w(angular_I.X(), angular_I.Y(), angular_I.Z());
  AddNoise(&a, &w, dt);
  const auto sec = std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
  imu_message_.mutable_header()->mutable_stamp()->set_sec(sec.count());
  imu_message_.mutable_header()->mutable_stamp()->set_nsec(
      std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime - sec).count());
  auto *q = imu_message_.mutable_orientation();
  q->set_w(rotation.W()); q->set_x(rotation.X()); q->set_y(rotation.Y()); q->set_z(rotation.Z());
  auto *linear = imu_message_.mutable_linear_acceleration();
  linear->set_x(a.x()); linear->set_y(a.y()); linear->set_z(a.z());
  auto *rate = imu_message_.mutable_angular_velocity();
  rate->set_x(w.x()); rate->set_y(w.y()); rate->set_z(w.z());
  imu_pub_.Publish(imu_message_);
}
}  // namespace gazebo
GZ_ADD_PLUGIN(gazebo::GazeboImuPlugin, gz::sim::System,
    gazebo::GazeboImuPlugin::ISystemConfigure, gazebo::GazeboImuPlugin::ISystemPreUpdate,
    gazebo::GazeboImuPlugin::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(gazebo::GazeboImuPlugin, "imu_plugin")
GZ_ADD_PLUGIN_ALIAS(gazebo::GazeboImuPlugin, "gazebo::GazeboImuPlugin")
