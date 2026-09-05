#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <vectortricopter/world.hpp>
#include <vectortricopter/vehicle.hpp>

namespace tilt {
namespace tricopter {

namespace {
rclcpp::Node::SharedPtr node_instance;
std::mutex node_mutex;
}

rclcpp::Node::SharedPtr GetNode() {
    std::lock_guard<std::mutex> lock(node_mutex);
    if (!node_instance) {
        rclcpp::NodeOptions options;
        options.automatically_declare_parameters_from_overrides(true);
        node_instance = std::make_shared<rclcpp::Node>(
            "tricopter_control_node", options);
    }
    return node_instance;
}

void ResetNode() {
    std::lock_guard<std::mutex> lock(node_mutex);
    node_instance.reset();
}

double GetCurrentTimeSeconds() {
    return GetNode()->get_clock()->now().seconds();
}

Vehicle& GetVehicle() {
    return Vehicle::GetInstance();
}

void SetVehicleTrajectory(const TrajectorySetpoint& setpoint) {
    GetVehicle().SetTrajectorySetpoint(setpoint);
}

}
}
