#include "uav_pid_control_node.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace {
using std::placeholders::_1;

std::string Ros1ParameterKeyToRos2(const std::string& key) {
    std::string converted = key;
    std::replace(converted.begin(), converted.end(), '/', '.');
    return converted;
}
}  // namespace

UAVPIDControllerNode::UAVPIDControllerNode(
    const rclcpp::Node::SharedPtr& n1,
    const rclcpp::Node::SharedPtr& _private_nh)
    : expect_Q(1.0f, 0.0f, 0.0f, 0.0f)
    , expect_position(0.0f, 0.0f, 3.0f)
    , expect_EulerAngle(0.0f, 0.0f, 0.0f)
    , position_gain(0.0f, 0.0f, 0.0f)
    , attitude_gain(0.0f, 0.0f, 0.0f)
    , output_angular_accel(0.0f, 0.0f, 0.0f)
    , dt(0.01)
    , n1(n1)
    , _private_nh(_private_nh)
{
    state_rpy.setZero();
    state_Q.setIdentity();
    state_pos.setZero();
    state_vel.setZero();
    state_angle_rate.setZero();
    Force_IN_Navigation.setZero();
    Force_IN_Body.setZero();
    Momrnt_IN_Body.setZero();

    odemetry_sub = n1->create_subscription<nav_msgs::msg::Odometry>(
        "/iwp/imu", rclcpp::QoS(10),
        std::bind(&UAVPIDControllerNode::OdometryCallback, this, _1));

    command_pose_sub = n1->create_subscription<nav_msgs::msg::Odometry>(
        "/iwp/command/pose", rclcpp::QoS(1),
        std::bind(&UAVPIDControllerNode::CommandPosCallback, this, _1));

    motor_pub = n1->create_publisher<mav_msgs::msg::Actuators>(
        "/iwp/gazebo/command/prop_speed", rclcpp::QoS(10));

    servo_pub = n1->create_publisher<mav_msgs::msg::Actuators>(
        "/iwp/gazebo/command/tilt_pos", rclcpp::QoS(10));

    _state_r_pub = n1->create_publisher<std_msgs::msg::Float32>("state/roll", 1);

    _state_p_pub = n1->create_publisher<std_msgs::msg::Float32>("state/pitch", 1);

    _state_y_pub = n1->create_publisher<std_msgs::msg::Float32>("state/yaw", 1);

    _state_r_der_pub = n1->create_publisher<std_msgs::msg::Float32>("state/roll_der", 1);

    _state_p_der_pub = n1->create_publisher<std_msgs::msg::Float32>("state/pitch_der", 1);

    _state_y_der_pub = n1->create_publisher<std_msgs::msg::Float32>("state/yaw_der", 1);

    _state_u_pub = n1->create_publisher<std_msgs::msg::Float32>("state/xpos", 1);

    _state_v_pub = n1->create_publisher<std_msgs::msg::Float32>("state/ypos", 1);

    _state_w_pub = n1->create_publisher<std_msgs::msg::Float32>("state/zpos", 1);

    _state_u_der_pub = n1->create_publisher<std_msgs::msg::Float32>("state/xvel", 1);

    _state_v_der_pub = n1->create_publisher<std_msgs::msg::Float32>("state/yvel", 1);

    _state_w_der_pub = n1->create_publisher<std_msgs::msg::Float32>("state/zvel", 1);

    InitParam();
}

Eigen::MatrixXd UAVPIDControllerNode::Pinv(Eigen::MatrixXd A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // M=USV*
    double Pinvtoler = 1.e-8; // tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row, col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues(); //奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i < k; ++i) {
        if (singularValues_inv(i) > Pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else
            singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose()); // X=VS+U*

    return X;
}

void UAVPIDControllerNode::CommandPosCallback(
    nav_msgs::msg::Odometry::ConstSharedPtr odometry)
{
    if (!odometry) return;
    expect_position(0) = odometry->pose.pose.position.x;
    expect_position(1) = odometry->pose.pose.position.y;
    expect_position(2) = odometry->pose.pose.position.z;

    // Eigen::Quaterniond quat;
    // quat.w() = odometry.pose.pose.orientation.w;
    // quat.x() = odometry.pose.pose.orientation.x;
    // quat.y() = odometry.pose.pose.orientation.y;
    // quat.z() = odometry.pose.pose.orientation.z;

    // expect_EulerAngle(0) = quat.
    // expect_EulerAngle(2) = pose.yaw;
}

void UAVPIDControllerNode::OdometryCallback(
    nav_msgs::msg::Odometry::ConstSharedPtr odometry)
{
    if (!odometry) return;
    state_Q.w() = odometry->pose.pose.orientation.w;
    state_Q.x() = odometry->pose.pose.orientation.x;
    state_Q.y() = odometry->pose.pose.orientation.y;
    state_Q.z() = odometry->pose.pose.orientation.z;

    state_pos(0) = odometry->pose.pose.position.x;
    state_pos(1) = odometry->pose.pose.position.y;
    state_pos(2) = odometry->pose.pose.position.z;

    state_vel(0) = odometry->twist.twist.linear.x;
    state_vel(1) = odometry->twist.twist.linear.y;
    state_vel(2) = odometry->twist.twist.linear.z;

    state_vel = state_Q.inverse() * state_vel;

    state_angle_rate(0) = odometry->twist.twist.angular.x;
    state_angle_rate(1) = odometry->twist.twist.angular.y;
    state_angle_rate(2) = odometry->twist.twist.angular.z;

    std::cout << "state_Q:" << state_Q.matrix().eulerAngles(2, 1, 0) << std::endl;
}

void UAVPIDControllerNode::GetRosParameter(
    const rclcpp::Node::SharedPtr& nh, const std::string& key,
    const float& default_value, float* value)
{
    if (value == nullptr || !nh) return;
    const std::string ros2_key = Ros1ParameterKeyToRos2(key);
    if (!nh->has_parameter(ros2_key)) {
        try {
            nh->declare_parameter<double>(ros2_key, default_value);
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
            // A parameter supplied by a launch YAML file may already be
            // declared when NodeOptions processes its overrides.
        }
    }
    double parameter_value = default_value;
    if (nh->get_parameter(ros2_key, parameter_value)) {
        *value = static_cast<float>(parameter_value);
    } else {
        RCLCPP_WARN(nh->get_logger(),
                    "[rosparam]: could not find parameter %s, setting to default: %f",
                    key.c_str(), static_cast<double>(default_value));
        *value = default_value;
    }
}

void UAVPIDControllerNode::InitParam()
{
    float gain, kp, ki, kd, integral_limit, output_limit, dt;
    GetRosParameter(_private_nh, "pid_dt", 0.01, &dt);

    GetRosParameter(_private_nh, "position_gain/x", 0.0, &gain);
    position_gain(0) = gain;
    GetRosParameter(_private_nh, "position_gain/y", 0.0, &gain);
    position_gain(1) = gain;
    GetRosParameter(_private_nh, "position_gain/z", 0.0, &gain);
    position_gain(2) = gain;

    GetRosParameter(_private_nh, "attitude_gain/x", 0.0, &gain);
    attitude_gain(0) = gain;
    GetRosParameter(_private_nh, "attitude_gain/y", 0.0, &gain);
    attitude_gain(1) = gain;
    GetRosParameter(_private_nh, "attitude_gain/z", 0.0, &gain);
    attitude_gain(2) = gain;

    GetRosParameter(_private_nh, "pid_u/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_u/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_u/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_u/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_u/output_limit", 1, &output_limit);
    PID_Init(&_pid_u, kp, ki, kd, dt, output_limit, integral_limit);

    GetRosParameter(_private_nh, "pid_v/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_v/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_v/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_v/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_v/output_limit", 1, &output_limit);
    PID_Init(&_pid_v, kp, ki, kd, dt, output_limit, integral_limit);

    GetRosParameter(_private_nh, "pid_w/kp", 1, &kp);
    GetRosParameter(_private_nh, "pid_w/ki", 0.01, &ki);
    GetRosParameter(_private_nh, "pid_w/kd", 0.55, &kd);
    GetRosParameter(_private_nh, "pid_w/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_w/output_limit", 1, &output_limit);
    PID_Init(&_pid_w, kp, ki, kd, dt, output_limit, integral_limit);

    GetRosParameter(_private_nh, "pid_p/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_p/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_p/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_p/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_p/output_limit", 1, &output_limit);
    PID_Init(&_pid_p, kp, ki, kd, dt, output_limit, integral_limit);

    GetRosParameter(_private_nh, "pid_q/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_q/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_q/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_q/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_q/output_limit", 1, &output_limit);
    PID_Init(&_pid_q, kp, ki, kd, dt, output_limit, integral_limit);

    GetRosParameter(_private_nh, "pid_r/kp", 1.8, &kp);
    GetRosParameter(_private_nh, "pid_r/ki", 0.025, &ki);
    GetRosParameter(_private_nh, "pid_r/kd", 0.88, &kd);
    GetRosParameter(_private_nh, "pid_r/integral_limit", 2, &integral_limit);
    GetRosParameter(_private_nh, "pid_r/output_limit", 1, &output_limit);
    PID_Init(&_pid_r, kp, ki, kd, dt, output_limit, integral_limit);

    float kf, km, l;
    ////从 rosparam 读取飞行器参数 初始化控制分配矩阵
    GetRosParameter(_private_nh, "vehicle/kf", 1.7088e-05, &kf);
    GetRosParameter(_private_nh, "vehicle/km", 1.7088e-05 * 0.016, &km);
    GetRosParameter(_private_nh, "vehicle/l", 0.355, &l);
    GetRosParameter(_private_nh, "vehicle/mass", 2.274, &_mass);
    GetRosParameter(_private_nh, "vehicle/Ix", 0.021968, &_Ix);
    GetRosParameter(_private_nh, "vehicle/Iy", 0.021968, &_Iy);
    GetRosParameter(_private_nh, "vehicle/Iz", 0.042117, &_Iz);

    Eigen::Matrix<double, 6, 12> angular_to_m;

    angular_to_m << 0, -kf / 2, 0, -kf, 0, -kf / 2, 0, kf / 2, 0, kf, 0, kf / 2, 0, -(sqrt(3) * kf) / 2, 0, 0, 0,
        (sqrt(3) * kf) / 2, 0, (sqrt(3) * kf) / 2, 0, 0, 0, -(sqrt(3) * kf) / 2, kf, 0, kf, 0, kf, 0, kf, 0, kf, 0, kf,
        0, -(kf * l) / 2, km / 2, -kf * l, -km, -(kf * l) / 2, km / 2, (kf * l) / 2, km / 2, kf * l, -km, (kf * l) / 2,
        km / 2, -(sqrt(3) * kf * l) / 2, (sqrt(3) * km) / 2, 0, 0, (sqrt(3) * kf * l) / 2, -(sqrt(3) * km) / 2,
        (sqrt(3) * kf * l) / 2, (sqrt(3) * km) / 2, 0, 0, -(sqrt(3) * kf * l) / 2, -(sqrt(3) * km) / 2, -km, -kf * l,
        km, -kf * l, -km, -kf * l, km, -kf * l, -km, -kf * l, km, -kf * l;

    _allocationMatrix = Pinv(angular_to_m);
}

void UAVPIDControllerNode::EulerBasedControl(double dt)
{
    Eigen::Vector3d expect_velocity(0, 0, 0);
    Eigen::Vector3d expect_angular(0, 0, 0);
    Eigen::Vector3d expect_angular_acc(0, 0, 0);
    Eigen::Vector3d expect_rpy(0, 0, 0);
    Eigen::Vector3d expect_accel(0, 0, 0);

    ////////////位置控制 双环PID///////////////////

    expect_velocity = position_gain.cwiseProduct(expect_position - state_pos);

    expect_accel(0) = PID_calculate(&_pid_u, expect_velocity(0), state_vel(0));
    expect_accel(1) = PID_calculate(&_pid_v, expect_velocity(1), state_vel(1));
    expect_accel(2) = PID_calculate(&_pid_w, expect_velocity(2), state_vel(2));

    if (isnan(expect_accel(0)))
        expect_accel(0) = 0;
    if (isnan(expect_accel(1)))
        expect_accel(1) = 0;
    if (isnan(expect_accel(2)))
        expect_accel(2) = 0;

    Force_IN_Navigation = expect_accel * 9.8f * _mass;
    Force_IN_Navigation(2) += 9.8f * _mass;

    ///////////////姿态控制 双环PID///////////
    expect_angular << attitude_gain(0) * (expect_EulerAngle(0) - state_rpy(0) * 57.3f),
        attitude_gain(1) * (expect_EulerAngle(1) - state_rpy(1) * 57.3f),
        attitude_gain(2) * (expect_EulerAngle(2) - state_rpy(2) * 57.3f);

    expect_angular = expect_angular / 57.3f;

    expect_angular_acc << PID_calculate(&_pid_p, expect_angular(0), state_angle_rate(0)),
        PID_calculate(&_pid_q, expect_angular(1), state_angle_rate(1)),
        PID_calculate(&_pid_r, expect_angular(2), state_angle_rate(2));

    if (state_pos(2) < 1) {
        expect_angular_acc(0) = 0;
        expect_angular_acc(1) = 0;
        expect_angular_acc(2) = 0;
    }

    // if(isnan(expect_angular_acc(0)))
    //     expect_angular_acc(0) = 0;
    // if(isnan(expect_angular_acc(1)))
    //     expect_angular_acc(1) = 0;
    // if(isnan(expect_angular_acc(2)))
    //     expect_angular_acc(2) = 0;

    // std::printf("expect_angular_acc out(1)=%f,out(2)=%f,out(3)=%f\n\n", expect_angular_acc(0), expect_angular_acc(1),
    //     expect_angular_acc(2));

    Momrnt_IN_Body = expect_angular_acc;
}

void UAVPIDControllerNode::ControlAllocation()
{
    Eigen::Matrix3d eRb = state_Q.toRotationMatrix();
    Eigen::Matrix3d bRe = eRb.transpose();
    Force_IN_Body = bRe * Force_IN_Navigation;

    Eigen::Matrix<double, 6, 1> ForceMomrnt_IN_Body;
    ForceMomrnt_IN_Body << Force_IN_Body, Momrnt_IN_Body;

    Eigen::Matrix<double, 12, 1> VirtualControl;
    VirtualControl = _allocationMatrix * ForceMomrnt_IN_Body;

    //////////////通过控制分配将机体系期望力和力矩转化为电机转速和倾转角
    double motor[4]; //
    double alpha[4]; //

    motor[0] = sqrt(sqrt(VirtualControl(0) * VirtualControl(0) + VirtualControl(1) * VirtualControl(1)));
    alpha[0] = atan2(VirtualControl(1), VirtualControl(0));

    motor[1] = sqrt(sqrt(VirtualControl(2) * VirtualControl(2) + VirtualControl(3) * VirtualControl(3)));
    alpha[1] = atan2(VirtualControl(3), VirtualControl(2));

    motor[2] = sqrt(sqrt(VirtualControl(4) * VirtualControl(4) + VirtualControl(5) * VirtualControl(5)));
    alpha[2] = atan2(VirtualControl(5), VirtualControl(4));

    motor[3] = sqrt(sqrt(VirtualControl(6) * VirtualControl(6) + VirtualControl(7) * VirtualControl(7)));
    alpha[3] = atan2(VirtualControl(7), VirtualControl(6));

    SetMotor(motor);
    SetServo(alpha);
}

Eigen::Vector3d& UAVPIDControllerNode::getRPY() { return state_rpy; }

Eigen::Quaterniond& UAVPIDControllerNode::getQuaterniond() { return state_Q; }

void UAVPIDControllerNode::showParam()
{
    std::cout << "FM_b_to_actuator allocation:\n" << _allocationMatrix << std::endl;
}

void UAVPIDControllerNode::QtoEuler(Eigen::Vector3d& rpy, const Eigen::Quaterniond& Q)
{
    rpy(0) = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), 1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y()));
    rpy(1) = asin(2 * (Q.w() * Q.y() - Q.x() * Q.z()));
    rpy(2) = atan2(2 * (Q.w() * Q.z() + Q.y() * Q.x()), 1 - 2 * (Q.z() * Q.z() + Q.y() * Q.y()));
}

void UAVPIDControllerNode::SendRPY()
{
    std_msgs::msg::Float32 msg;
    msg.data = state_rpy(0) / M_PI * 180;
    _state_r_pub->publish(msg);
    msg.data = state_rpy(1) / M_PI * 180;
    _state_p_pub->publish(msg);
    msg.data = state_rpy(2) / M_PI * 180;
    _state_y_pub->publish(msg);

    msg.data = state_angle_rate(0) / M_PI * 180;
    _state_r_der_pub->publish(msg);
    msg.data = state_angle_rate(1) / M_PI * 180;
    _state_p_der_pub->publish(msg);
    msg.data = state_angle_rate(2) / M_PI * 180;
    _state_y_der_pub->publish(msg);

    msg.data = state_pos(0);
    _state_u_pub->publish(msg);
    msg.data = state_pos(1);
    _state_v_pub->publish(msg);
    msg.data = state_pos(2);
    _state_w_pub->publish(msg);

    msg.data = state_vel(0);
    _state_u_der_pub->publish(msg);
    msg.data = state_vel(1);
    _state_v_der_pub->publish(msg);
    msg.data = state_vel(2);
    _state_w_der_pub->publish(msg);
}

void UAVPIDControllerNode::SetMotor(double motor[])
{
    motor_msg.angular_velocities.clear();

    motor_msg.angular_velocities.push_back(motor[0]);

    motor_msg.angular_velocities.push_back(motor[1]);

    motor_msg.angular_velocities.push_back(motor[2]);

    motor_msg.angular_velocities.push_back(motor[3]);

    motor_pub->publish(motor_msg);
}

void UAVPIDControllerNode::SetServo(double servo[])
{
    servo_msg.angular_velocities.clear();

    servo_msg.angular_velocities.push_back(servo[0]);

    servo_msg.angular_velocities.push_back(servo[1]);

    servo_msg.angular_velocities.push_back(servo[2]);

    servo_msg.angular_velocities.push_back(servo[3]);

    servo_pub->publish(servo_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto nh = std::make_shared<rclcpp::Node>("uav_pid_control_node", options);

    UAVPIDControllerNode uavnode(nh, nh);

    rclcpp::WallRate loop_rate(100.0);

    uavnode.showParam();

    while (rclcpp::ok()) {

        uavnode.EulerBasedControl(0.01);

        uavnode.ControlAllocation();

        rclcpp::spin_some(nh);

        uavnode.QtoEuler(uavnode.getRPY(), uavnode.getQuaterniond());

        uavnode.SendRPY();

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
