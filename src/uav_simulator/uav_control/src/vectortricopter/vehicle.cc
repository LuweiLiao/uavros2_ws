#include <vectortricopter/vehicle.hpp>

#include <cassert>
#include <functional>
#include <string>

#include <vectortricopter/scheduler.hpp>
#include <vectortricopter/world.hpp>

using std::placeholders::_1;

namespace tilt {
namespace tricopter {

Vehicle *Vehicle::globalVehicleInstance = nullptr;

Vehicle::Vehicle() {
    LoadParametersFromNH();

    const auto node = GetNode();
    _odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "/gazebo/model_states", rclcpp::QoS(10),
        std::bind(&Vehicle::ModelStatesCallback, this, _1));
    _imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/tilt_tricopter/imu", rclcpp::QoS(10),
        std::bind(&Vehicle::IMUCallback, this, _1));

    StartControlLoop();
}

Vehicle& Vehicle::GetInstance() {
    if (!globalVehicleInstance) {
        globalVehicleInstance = new Vehicle();
    }

    assert(globalVehicleInstance != nullptr);

    return *globalVehicleInstance;
}

void Vehicle::ShutdownInstance() {
    delete globalVehicleInstance;
    globalVehicleInstance = nullptr;
}

void Vehicle::LoadParametersFromNH() {
    const auto nh = GetNode();

    Parameter param;

    LoadPosCtrlParameters(nh, param.pos_ctrl_param);
    LoadAttCtrlParameters(nh, param.att_ctrl_param);
    LoadRateCtrlParameters(nh, param.rate_ctrl_param);
    LoadCtrlAllocParameters(nh, param.ctrl_alloc_param);

    _pos_ctrl.Init(param.pos_ctrl_param);
    _att_ctrl.Init(param.att_ctrl_param);
    _rate_ctrl.Init(param.rate_ctrl_param);
    _control_allocator.Init(param.ctrl_alloc_param);
}

#define LOAD_PARAM_VALUE(namespace, name, default) \
    param.name = GetParameter(nh, #namespace "." #name, default); \
    assert(param.name >= 0)

template <typename T>
T GetParameter(const rclcpp::Node::SharedPtr& node, const std::string& name,
               const T& default_value) {
    if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
    }
    T value = default_value;
    node->get_parameter(name, value);
    return value;
}

void Vehicle::LoadPosCtrlParameters(const rclcpp::Node::SharedPtr& nh,
                                    TriPositionControl::Parameter& param) {
    LOAD_PARAM_VALUE(pos_ctrl, kpx, 5);
    LOAD_PARAM_VALUE(pos_ctrl, kpy, 5);
    LOAD_PARAM_VALUE(pos_ctrl, kpz, 5);
    LOAD_PARAM_VALUE(pos_ctrl, kvx, 1.5);
    LOAD_PARAM_VALUE(pos_ctrl, kvy, 1.5);
    LOAD_PARAM_VALUE(pos_ctrl, kvz, 1.5);

    LOAD_PARAM_VALUE(pos_ctrl, hover_thrust, 0.5);
}

void Vehicle::LoadAttCtrlParameters(const rclcpp::Node::SharedPtr& nh,
                                    TriAttitudeControl::Parameter& param){
    LOAD_PARAM_VALUE(att_ctrl, roll_gain, 1);
    LOAD_PARAM_VALUE(att_ctrl, pitch_gain, 1);
    LOAD_PARAM_VALUE(att_ctrl, yaw_gain, 1);
}

void Vehicle::LoadRateCtrlParameters(const rclcpp::Node::SharedPtr& nh,
                                     TriRateControl::Parameter& param) {
    LOAD_PARAM_VALUE(rate_ctrl, kroll, 1);
    LOAD_PARAM_VALUE(rate_ctrl, kpitch, 1);
    LOAD_PARAM_VALUE(rate_ctrl, kyaw, 1);

    LOAD_PARAM_VALUE(rate_ctrl, ki_roll, 0.12);
    LOAD_PARAM_VALUE(rate_ctrl, ki_pitch, 0.12);
    LOAD_PARAM_VALUE(rate_ctrl, ki_yaw, 0.12);

    LOAD_PARAM_VALUE(rate_ctrl, kp_roll, 5);
    LOAD_PARAM_VALUE(rate_ctrl, kp_pitch, 5);
    LOAD_PARAM_VALUE(rate_ctrl, kp_yaw, 5);
}

void Vehicle::LoadCtrlAllocParameters(const rclcpp::Node::SharedPtr& nh,
                                      TriControlAllocator::Parameter& param) {
    LOAD_PARAM_VALUE(control_allocator, alpha, 1);

    param.rotor_topic = GetParameter<std::string>(
        nh, "control_allocator.rotor_topic", "/gazebo/command/prop_speed");
    param.servo_topic = GetParameter<std::string>(
        nh, "control_allocator.servo_topic", "/gazebo/command/tilt1_pos");
}

#undef LOAD_PARAM_VALUE

void Vehicle::StartControlLoop() {
    std::vector<Scheduler::Task> tasks = {
        SCHED_TASK_CLASS(&_pos_ctrl, TriPositionControl, Run, 50),
        SCHED_TASK_CLASS(&_att_ctrl, TriAttitudeControl, Run, 100),
        SCHED_TASK_CLASS(&_rate_ctrl, TriRateControl, Run, 500),
        SCHED_TASK_CLASS(&_control_allocator, TriControlAllocator, Run, 500)
    };

    Scheduler& scheduler = Scheduler::GetInstance();

    scheduler.Init(tasks);
}

void Vehicle::SetTrajectorySetpoint(const TrajectorySetpoint& setpoint) {
    _pos_ctrl.SetTrajectorySetpoint(setpoint);
}

void Vehicle::SetAttitudeSetpoint(const AttitudeSetpoint& setpoint) {
    _att_ctrl.SetAttitudeSetpoint(setpoint);
}

void Vehicle::SetRateSetpoint(const RateSetpoint& setpoint) {
    _rate_ctrl.SetRateSetpoint(setpoint);
}

void Vehicle::SetCASetpoint(const CASetpoint& setpoint) {
    _control_allocator.SetCASetpoint(setpoint);
}

void Vehicle::ModelStatesCallback(
    nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    if (!msg) {
        return;
    }
    std::unique_lock<std::mutex> lock(_odom_mutex);

    _position.x() = msg->pose.pose.position.x;
    _position.y() = msg->pose.pose.position.y;
    _position.z() = msg->pose.pose.position.z;

    _velocity.x() = msg->twist.twist.linear.x;
    _velocity.y() = msg->twist.twist.linear.y;
    _velocity.z() = msg->twist.twist.linear.z;

    _attitude.w() = msg->pose.pose.orientation.w;
    _attitude.x() = msg->pose.pose.orientation.x;
    _attitude.y() = msg->pose.pose.orientation.y;
    _attitude.z() = msg->pose.pose.orientation.z;
}

void Vehicle::IMUCallback(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    if (!msg) {
        return;
    }
    std::unique_lock<std::mutex> lock(_imu_mutex);

    _acceleration.x() = msg->linear_acceleration.x;
    _acceleration.y() = msg->linear_acceleration.y;
    _acceleration.z() = msg->linear_acceleration.z;

    _omega.x() = msg->angular_velocity.x;
    _omega.y() = msg->angular_velocity.y;
    _omega.z() = msg->angular_velocity.z;

    _current_time.store(
        static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) * 1e-9);
}

}
}
