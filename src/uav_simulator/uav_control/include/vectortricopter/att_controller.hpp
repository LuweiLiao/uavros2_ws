#pragma once

#include <Eigen/Dense>

#include <vectortricopter/msg/setpoints.hpp>

namespace tilt {
namespace tricopter {

class TriAttitudeControl {
public:
    struct Parameter {
        double roll_gain;
        double pitch_gain;
        double yaw_gain;
    };

    void Init(const Parameter& param);

    void SetAttitudeSetpoint(const AttitudeSetpoint& setpoint);

    void Run();
private:
    Parameter _params;

    Eigen::Vector3d _angle_gain;

    AttitudeSetpoint _att_setpoint;

    bool _att_initialized{false};
};

}
}