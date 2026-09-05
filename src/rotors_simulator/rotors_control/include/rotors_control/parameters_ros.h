#ifndef INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_
#define INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_

#include <algorithm>
#include <cassert>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rotors_control/parameters.h"

namespace rotors_control {

inline std::string Ros2ParameterName(std::string key) {
  std::replace(key.begin(), key.end(), '/', '.');
  return key;
}

template<typename T> inline void GetRosParameter(rclcpp::Node& node,
                                                 const std::string& key,
                                                 const T& default_value,
                                                 T* value) {
  assert(value != nullptr);
  const std::string name = Ros2ParameterName(key);
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  if (!node.get_parameter(name, *value)) {
    RCLCPP_WARN(node.get_logger(),
                "[rosparam]: could not find parameter %s, setting to default",
                name.c_str());
    *value = default_value;
  }
}

inline void GetRotorConfiguration(rclcpp::Node& node,
                                  RotorConfiguration* rotor_configuration) {
  assert(rotor_configuration != nullptr);
  for (unsigned int i = 0; i < 32; ++i) {
    const std::string prefix = "rotor_configuration." + std::to_string(i);
    const bool present = node.has_parameter(prefix + ".angle") ||
                         node.has_parameter(prefix + ".arm_length") ||
                         node.has_parameter(prefix + ".rotor_force_constant") ||
                         node.has_parameter(prefix + ".rotor_moment_constant") ||
                         node.has_parameter(prefix + ".direction");
    if (!present) {
      break;
    }
    if (i == 0) {
      rotor_configuration->rotors.clear();
    }
    Rotor rotor;
    GetRosParameter(node, prefix + ".angle", rotor.angle, &rotor.angle);
    GetRosParameter(node, prefix + ".arm_length", rotor.arm_length,
                    &rotor.arm_length);
    GetRosParameter(node, prefix + ".rotor_force_constant",
                    rotor.rotor_force_constant, &rotor.rotor_force_constant);
    GetRosParameter(node, prefix + ".rotor_moment_constant",
                    rotor.rotor_moment_constant, &rotor.rotor_moment_constant);
    GetRosParameter(node, prefix + ".direction", rotor.direction,
                    &rotor.direction);
    rotor_configuration->rotors.push_back(rotor);
  }
}

inline void GetVehicleParameters(rclcpp::Node& node,
                                 VehicleParameters* vehicle_parameters) {
  assert(vehicle_parameters != nullptr);
  GetRosParameter(node, "mass",
                  vehicle_parameters->mass_,
                  &vehicle_parameters->mass_);
  GetRosParameter(node, "inertia/xx",
                  vehicle_parameters->inertia_(0, 0),
                  &vehicle_parameters->inertia_(0, 0));
  GetRosParameter(node, "inertia/xy",
                  vehicle_parameters->inertia_(0, 1),
                  &vehicle_parameters->inertia_(0, 1));
  vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
  GetRosParameter(node, "inertia/xz",
                  vehicle_parameters->inertia_(0, 2),
                  &vehicle_parameters->inertia_(0, 2));
  vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
  GetRosParameter(node, "inertia/yy",
                  vehicle_parameters->inertia_(1, 1),
                  &vehicle_parameters->inertia_(1, 1));
  GetRosParameter(node, "inertia/yz",
                  vehicle_parameters->inertia_(1, 2),
                  &vehicle_parameters->inertia_(1, 2));
  vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
  GetRosParameter(node, "inertia/zz",
                  vehicle_parameters->inertia_(2, 2),
                  &vehicle_parameters->inertia_(2, 2));
  GetRotorConfiguration(node, &vehicle_parameters->rotor_configuration_);
}
}

#endif /* INCLUDE_ROTORS_CONTROL_PARAMETERS_ROS_H_ */
