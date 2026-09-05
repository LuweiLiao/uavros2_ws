#pragma once

#include <string>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <mav_msgs/msg/actuators.hpp>

#include <Eigen/Dense>

#include <vectortricopter/msg/setpoints.hpp>

namespace tilt {
namespace tricopter {

class TriControlAllocator {
public:
    struct Parameter {
        double alpha;

        std::string rotor_topic;
        std::string servo_topic;
    };

    void Init(const Parameter& param);

    void Run();

    void SetCASetpoint(const CASetpoint& setpoint);

private:
    void CreateCAMatrix();

    void CreateSubsAndPubs();
private:
    Parameter _param;

    CASetpoint _ca_setpoint;
    std::atomic_bool _setpoint_initialized{false};
    std::mutex _setpoint_mutex;

    rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr _rotor_speed_pub;
    rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr _servo_position_pub;

    Eigen::MatrixXd _ca_matrix;
};

}
}
