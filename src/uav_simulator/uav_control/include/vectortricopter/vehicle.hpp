#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vectortricopter/pos_controller.hpp>
#include <vectortricopter/att_controller.hpp>
#include <vectortricopter/rate_controller.hpp>
#include <vectortricopter/control_allocator.hpp>

#include <vectortricopter/msg/setpoints.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace tilt {
namespace tricopter {

class Vehicle {
public:
    struct Parameter {
        TriPositionControl::Parameter pos_ctrl_param;
        TriAttitudeControl::Parameter att_ctrl_param;
        TriRateControl::Parameter rate_ctrl_param;
        TriControlAllocator::Parameter ctrl_alloc_param;
    };


    // Get the global instance of the vehicle
    static Vehicle& GetInstance();

    // Destroy the process-lifetime singleton and its ROS entities cleanly.
    // The scheduler must be stopped before calling this method.
    static void ShutdownInstance();

    // Get the current position of the vehicle
    Eigen::Vector3d GetVehiclePosition() {
        std::unique_lock<std::mutex> lock(_odom_mutex);
        return _position;
    }

    // Get the current velocity of the vehicle
    Eigen::Vector3d GetVehicleVelocity() {
        std::unique_lock<std::mutex> lock(_odom_mutex);
        return _velocity;
    }

    // Get the current orientation of the vehicle
    Eigen::Quaterniond GetVehicleAttitude() {
        std::unique_lock<std::mutex> lock(_odom_mutex);
        return _attitude;
    }

    // Get the current acceleration of the vehicle
    Eigen::Vector3d GetVehicleAcceleration() {
        std::unique_lock<std::mutex> lock(_imu_mutex);
        return _acceleration;
    }

    // Get the current angular velocity of the vehicle
    Eigen::Vector3d GetAngularVelocity() {
        std::unique_lock<std::mutex> lock(_imu_mutex);
        return _omega;
    }

    double GetCurrentTime() const {
        return _current_time.load();
    }

    void SetTrajectorySetpoint(const TrajectorySetpoint& setpoint);

    void SetAttitudeSetpoint(const AttitudeSetpoint& setpoint);

    void SetRateSetpoint(const RateSetpoint& setpoint);

    void SetCASetpoint(const CASetpoint& setpoint);
private:
    // Thr global instance of the vehicle
    static Vehicle *globalVehicleInstance;

    TriPositionControl _pos_ctrl;
    TriAttitudeControl _att_ctrl;
    TriRateControl _rate_ctrl;
    TriControlAllocator _control_allocator;
private:
    // Disable construct and copy from output
    Vehicle();

    Vehicle(const Vehicle&) = delete;
    Vehicle(Vehicle&&) = delete;
    Vehicle& operator=(const Vehicle&) = delete;
    Vehicle& operator=(Vehicle&&) = delete;

    void LoadParametersFromNH();

    void LoadPosCtrlParameters(const rclcpp::Node::SharedPtr& nh,
                               TriPositionControl::Parameter& param);
    void LoadAttCtrlParameters(const rclcpp::Node::SharedPtr& nh,
                               TriAttitudeControl::Parameter& param);
    void LoadRateCtrlParameters(const rclcpp::Node::SharedPtr& nh,
                                TriRateControl::Parameter& param);
    void LoadCtrlAllocParameters(const rclcpp::Node::SharedPtr& nh,
                                 TriControlAllocator::Parameter& param);

    void StartControlLoop();

private:
    // Gazebo Classic published gazebo_msgs/ModelStates on this ROS 1 topic.
    // Gazebo Sim's official OdometryPublisher is bridged to the same topic
    // name as nav_msgs/Odometry; only the transport message changes.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;

    // Odometry 
    std::mutex _odom_mutex;
    Eigen::Vector3d _position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d _velocity{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond _attitude{Eigen::Quaterniond::Identity()};

    // IMU
    std::mutex _imu_mutex;
    Eigen::Vector3d _acceleration{Eigen::Vector3d::Zero()};
    Eigen::Vector3d _omega{Eigen::Vector3d::Zero()};
    std::atomic<double> _current_time{0.0};

    void ModelStatesCallback(
        nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void IMUCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg);
};

}
}
