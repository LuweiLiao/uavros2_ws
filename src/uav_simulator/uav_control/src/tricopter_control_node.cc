#include <vectortricopter/world.hpp>
#include <vectortricopter/msg/setpoints.hpp>
#include <vectortricopter/scheduler.hpp>
#include <vectortricopter/vehicle.hpp>

#include <mav_msgs/msg/trajectory_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

namespace {
std::atomic_bool stop_requested{false};

void SignalHandler(int) {
    stop_requested.store(true);
}
}  // namespace

static rclcpp::Subscription<mav_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_sub;

void TrajectorySetpointCallback(
    mav_msgs::msg::TrajectorySetpoint::ConstSharedPtr traj) {
    tilt::tricopter::TrajectorySetpoint setpoint;

    setpoint.x = traj->x;
    setpoint.y = traj->y;
    setpoint.z = traj->z;

    setpoint.vx = traj->vx;
    setpoint.vy = traj->vy;
    setpoint.vz = traj->vz;

    setpoint.ax = traj->ax;
    setpoint.ay = traj->ay;
    setpoint.az = traj->az;

    setpoint.yaw = traj->yaw;
    setpoint.pitch = traj->pitch;

    tilt::tricopter::SetVehicleTrajectory(setpoint);
}

int main(int argc, char** argv) {
    rclcpp::InitOptions init_options;
    // Defer context shutdown to the main thread.  The controller retains the
    // original worker-thread architecture, so its publishers must be stopped
    // before DDS entities are destroyed on SIGINT/SIGTERM.
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options);
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    auto node = tilt::tricopter::GetNode();

    trajectory_setpoint_sub = \
        node->create_subscription<mav_msgs::msg::TrajectorySetpoint>(
            "/tilt_tricopter/trajectory_setpoint", rclcpp::QoS(10),
            TrajectorySetpointCallback);

    (void)tilt::tricopter::GetVehicle();

    while (rclcpp::ok() && !stop_requested.load()) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    tilt::tricopter::Scheduler::GetInstance().Shutdown();
    tilt::tricopter::Vehicle::ShutdownInstance();
    trajectory_setpoint_sub.reset();
    node.reset();
    tilt::tricopter::ResetNode();
    rclcpp::shutdown();
    return 0;
}
