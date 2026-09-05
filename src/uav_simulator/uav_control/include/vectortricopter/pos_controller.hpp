#pragma once

#include <mutex>
#include <atomic>

#include <Eigen/Dense>

#include <vectortricopter/msg/setpoints.hpp>

namespace tilt {
namespace tricopter {

class TriPositionControl {
public:
    struct Parameter {
        double kpx;
        double kpy;
        double kpz;
        double kvx;
        double kvy;
        double kvz;

        double hover_thrust;
    };

    void Init(const Parameter& param);

    void SetTrajectorySetpoint(const TrajectorySetpoint& setpoint);

    void Run();

private:
    void CalculateAttitudeSetpoint(const Eigen::Vector3d& acc_sp);
private:
    Parameter _params;

    std::mutex _mutex;

    TrajectorySetpoint _trajectory_setpoint;

    std::atomic_bool _trajectory_initialized;

    Eigen::Vector3d _kp;
    Eigen::Vector3d _kv;

    double _hover_thrust;
};

}
}