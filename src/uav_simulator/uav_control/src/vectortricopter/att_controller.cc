#include <vectortricopter/att_controller.hpp>
#include <vectortricopter/vehicle.hpp>
#include <vectortricopter/world.hpp>

namespace tilt {
namespace tricopter {

void TriAttitudeControl::Init(const Parameter& param) {
    _params = param;

    _angle_gain = Eigen::Vector3d{
        param.roll_gain,
        param.pitch_gain,
        param.yaw_gain
    };

    _att_initialized = false;
}

void TriAttitudeControl::SetAttitudeSetpoint(const AttitudeSetpoint& setpoint) {
    _att_setpoint = setpoint;
    _att_initialized = true;
}

void TriAttitudeControl::Run() {
    if (!_att_initialized) return;

    Vehicle& vehicle = GetVehicle();

    Eigen::Quaterniond qstate = vehicle.GetVehicleAttitude();
    Eigen::Quaterniond qsetpoint(
        _att_setpoint.qw,
        _att_setpoint.qx,
        _att_setpoint.qy,
        _att_setpoint.qz
    );

    Eigen::Vector3d thrust(
        _att_setpoint.thrustx,
        _att_setpoint.thrusty,
        _att_setpoint.thrustz
    );

    // LOG(INFO) << "Thrust world is: " << thrust.transpose();

    Eigen::Vector3d thrust_body = qstate.inverse() * thrust;

    // LOG(INFO) << "Thrust body is: " << thrust_body.transpose();

    Eigen::Quaterniond qerror = qstate.inverse() * qsetpoint;

    // LOG(INFO) << "QState is: " << qstate.coeffs().transpose()
    //     << ". QSetpoint is: " << qsetpoint.coeffs().transpose();

    // LOG(INFO) << "QERROR is: " << qerror.coeffs().transpose();

    Eigen::Vector3d rate = 2 * _angle_gain.asDiagonal() * qerror.coeffs().head<3>();

    RateSetpoint rate_setpoint{
        .roll = rate.x(),
        .pitch = rate.y(),
        .yaw = rate.z(),
        .thrustx = thrust_body.x(),
        .thrusty = thrust_body.y(),
        .thrustz = thrust_body.z()
    };

    vehicle.SetRateSetpoint(rate_setpoint);
}

}
}
