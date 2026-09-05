#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <vectortricopter/msg/setpoints.hpp>

namespace tilt {
namespace tricopter {

class Vehicle;

// Get the current time in seconds
double GetCurrentTimeSeconds();

// The single ROS 2 node shared by the original controller components.  This
// replaces the ROS 1 private NodeHandle without introducing another package
// or changing the controller's public topics.
rclcpp::Node::SharedPtr GetNode();

// Release the shared node while the rclcpp context is still alive.  This is
// needed because the ROS 1 singleton was intentionally process-lifetime, but
// ROS 2 requires entities to be destroyed before shutdown.
void ResetNode();

// Get the current vehicle
Vehicle& GetVehicle();

// Set the current trajectory of the vehicle
void SetVehicleTrajectory(const TrajectorySetpoint& setpoint);

}
}
