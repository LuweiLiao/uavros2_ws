#pragma once

#include <memory>
#include <sstream>
#include "iostream"
#include "string"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mav_msgs/msg/actuators.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <math.h>
#include <std_msgs/msg/float32.hpp>

#include "pid_control.h"


class UAVPIDControllerNode
{

public:
    double Sgn(double val);

    Eigen::Vector3d &getRPY();
    Eigen::Quaterniond &getQuaterniond();
    void QtoEuler(Eigen::Vector3d &rpy, const Eigen::Quaterniond &Q);


    void SendRPY();

    void SetMotor(double motor[]);
    
    void SetServo(double servo[]);

    void GetRosParameter(const rclcpp::Node::SharedPtr &nh,
                         const std::string &key,
                         const float &default_value, float *value);

    void showParam();

    void InitParam();

    void QuaternionBasedControl(double dt);
    void EulerBasedControl(double dt);

    void ControlAllocation();

    void OdometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr odometry);

    void CommandPosCallback(nav_msgs::msg::Odometry::ConstSharedPtr odometry);


    // void CommandPosCallback(xfly::xfly_pose pose);

    Eigen::MatrixXd Pinv(Eigen::MatrixXd A);
    UAVPIDControllerNode(const rclcpp::Node::SharedPtr &n1,
                         const rclcpp::Node::SharedPtr &_private_nh);

private:
    Eigen::Vector3d state_rpy;        //vehicle current eular angle
    Eigen::Quaterniond state_Q;       //vehicle current Quaternion
    Eigen::Vector3d state_pos;        //vehicle current position
    Eigen::Vector3d state_vel;        //vehicle current body linear velocity
    Eigen::Vector3d state_angle_rate; //vehicle current body angle velocity

    Eigen::Quaterniond err_Q; //Error of Quaternion

    Eigen::Vector3d position_gain; //位置外环比例增益
    Eigen::Vector3d attitude_gain;

    Eigen::Vector3d expect_position; //位置期望
    Eigen::Vector3d expect_EulerAngle;

    Eigen::Vector3d Force_IN_Navigation, Force_IN_Body; //位置控制器输出：全局控制力期望
    Eigen::Vector3d Momrnt_IN_Body;

    Eigen::Vector3d output_angular_accel;

    Eigen::Quaterniond expect_Q; //vehicle expected Quaternion

    Eigen::Matrix<double, 12, 6> _allocationMatrix;

    mav_msgs::msg::Actuators motor_msg, servo_msg;  //vehicle publish motor message

    rclcpp::Node::SharedPtr n1,_private_nh;            //ros node

    rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub, servo_pub;
    // ros motor publisher

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odemetry_sub;
    //ros odemetry subscriber

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr command_pose_sub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        command_position_gain_sub, command_velocity_pid_sub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        command_attitude_gain_sub, command_angular_velocity_pid_sub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        _state_r_pub, _state_p_pub, _state_y_pub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        _state_r_der_pub, _state_p_der_pub, _state_y_der_pub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        _state_u_pub,_state_v_pub,_state_w_pub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        _state_u_der_pub,_state_v_der_pub,_state_w_der_pub;

    PID _pid_p, _pid_q, _pid_r; //角速度PID控制器

    PID _pid_u, _pid_v, _pid_w;

    float _mass, _Ix, _Iy, _Iz; //飞行器质量、惯量

    double dt; //control duty
};
