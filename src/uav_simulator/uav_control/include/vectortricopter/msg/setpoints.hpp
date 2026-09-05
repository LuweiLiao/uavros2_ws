#pragma once

namespace tilt {
namespace tricopter {

struct TrajectorySetpoint {
    double x{0};
    double y{0};
    double z{0};

    double vx{0};
    double vy{0};
    double vz{0};

    double ax{0};
    double ay{0};
    double az{0};

    double pitch{0};
    double yaw{0};
};

struct AttitudeSetpoint {
    double qw;
    double qx;
    double qy;
    double qz;

    // Thrust in world frame
    double thrustx;
    double thrusty;
    double thrustz;
};

struct RateSetpoint {
    double roll;
    double pitch;
    double yaw;

    // Thrust in body frame
    double thrustx;
    double thrusty;
    double thrustz;
};

struct CASetpoint {
    double tx;
    double ty;
    double tz;

    double mx;
    double my;
    double mz;
};

}
}