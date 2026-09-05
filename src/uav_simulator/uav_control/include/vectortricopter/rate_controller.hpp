#pragma once

#include <Eigen/Dense>

#include <vectortricopter/msg/setpoints.hpp>

namespace tilt {
namespace tricopter {

class TriRateControl {
public:
    struct Parameter {
        double kroll;
        double kpitch;
        double kyaw;

        double ki_roll;
        double ki_pitch;
        double ki_yaw;

        double kp_roll;
        double kp_pitch;
        double kp_yaw;
    };

    void Init(const Parameter& param);

    void SetRateSetpoint(const RateSetpoint& setpoint);

    void Run();
private:
    Parameter _params;

    Eigen::Vector3d _kgains;

    Eigen::Vector3d _kp;
    Eigen::Vector3d _ki;

    Eigen::Vector3d _error_int;

    RateSetpoint _rate_setpoint;
    bool _setpoint_initialized;

    double _dt;
    double _last_run_time;
};

}
}