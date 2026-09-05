#include <vectortricopter/rate_controller.hpp>
#include <vectortricopter/world.hpp>
#include <vectortricopter/vehicle.hpp>

namespace tilt {
namespace tricopter {

void TriRateControl::Init(const Parameter& param) {
    _params = param;

    _kgains = Eigen::Vector3d{
        param.kroll,
        param.kpitch,
        param.kyaw
    };

    _kp = Eigen::Vector3d{
        param.kp_roll,
        param.kp_pitch,
        param.kp_yaw
    };

    _ki = Eigen::Vector3d{
        param.ki_roll,
        param.ki_pitch,
        param.ki_yaw
    };

    _kp = _kgains.asDiagonal() * _kp;
    _ki = _kgains.asDiagonal() * _ki;

    _error_int.setZero();

    _setpoint_initialized = false;
    _last_run_time = 0;
}

void TriRateControl::SetRateSetpoint(const RateSetpoint& setpoint) {
    _rate_setpoint = setpoint;
    _setpoint_initialized = true;
}

void TriRateControl::Run() {
    if (!_setpoint_initialized) return;

    const double now = GetCurrentTimeSeconds();
    _dt = _last_run_time == 0 ? 0. : now - _last_run_time;
    _last_run_time = now;

    Vehicle& vehicle = GetVehicle();

    Eigen::Vector3d rate_setpoint{
        _rate_setpoint.roll,
        _rate_setpoint.pitch,
        _rate_setpoint.yaw
    };

    Eigen::Vector3d rate_state = vehicle.GetAngularVelocity();

    Eigen::Vector3d rate_error = rate_setpoint - rate_state;

    const double ratio = _error_int.norm() / M_PI * 180. / 400.;
    const double ifactor = std::max(
        0., 1. - ratio * ratio
    );

    _error_int += _ki.asDiagonal() * rate_error * _dt * ifactor;

    Eigen::Vector3d moment = _kp.asDiagonal() * rate_error +  _error_int;

    CASetpoint ca_setpoint {
        .tx = _rate_setpoint.thrustx,
        .ty = _rate_setpoint.thrusty,
        .tz = _rate_setpoint.thrustz,
        .mx = moment.x(),
        .my = moment.y(),
        .mz = moment.z()
    };

    vehicle.SetCASetpoint(ca_setpoint);
}

}
}