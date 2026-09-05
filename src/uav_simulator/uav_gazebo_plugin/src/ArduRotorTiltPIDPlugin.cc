/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <fcntl.h>
#include <functional>
#ifdef _WIN32
#include <Winsock2.h>
#include <Ws2def.h>
#include <Ws2ipdef.h>
#include <Ws2tcpip.h>
using raw_type = char;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
using raw_type = void;
#endif

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include "ArduRotorTiltPIDPlugin.hh"
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Filter.hh>
#include <mutex>
#include <sdf/sdf.hh>
#include <string>
#include <vector>

#define MAX_MOTORS 16

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ArduRotorTiltPIDPlugin)

/// \brief A servo packet.
struct ServoPacket {
    /// \brief Motor speed data.
    /// should rename to servo_command here and in ArduPilot SIM_Gazebo.cpp
    float motorSpeed[MAX_MOTORS] = { 0.0f };
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduPilot
struct fdmPacket {
    /// \brief packet timestamp
    double timestamp;

    /// \brief IMU angular velocity
    double imuAngularVelocityRPY[3];

    /// \brief IMU linear acceleration
    double imuLinearAccelerationXYZ[3];

    /// \brief IMU quaternion orientation
    double imuOrientationQuat[4];

    /// \brief Model velocity in NED frame
    double velocityXYZ[3];

    /// \brief Model position in NED frame
    double positionXYZ[3];
    /*  NOT MERGED IN MASTER YET
  /// \brief Model latitude in WGS84 system
  double latitude = 0.0;

  /// \brief Model longitude in WGS84 system
  double longitude = 0.0;

  /// \brief Model altitude from GPS
  double altitude = 0.0;

  /// \brief Model estimated from airspeed sensor (e.g. Pitot) in m/s
  double airspeed = 0.0;

  /// \brief Battery voltage. Default to -1 to use sitl estimator.
  double battery_voltage = -1.0;

  /// \brief Battery Current.
  double battery_current = 0.0;

  /// \brief Model rangefinder value. Default to -1 to use sitl rangefinder.
  double rangefinder = -1.0;
*/
};

/// \brief Control class
class Control {
    /// \brief Constructor
public:
    Control()
    {
        // most of these coefficients are not used yet.
        this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
        this->frequencyCutoff = this->kDefaultFrequencyCutoff;
        this->samplingRate = this->kDefaultSamplingRate;

        this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
    }

    /// \brief control id / channel
public:
    int channel = 0;

    /// \brief Next command to be applied to the propeller
public:
    double cmd = 0;

    /// \brief Velocity PID for motor control
public:
    common::PID pid;

    /// \brief Control type. Can be:
    /// VELOCITY control velocity of joint
    /// POSITION control position of joint
    /// EFFORT control effort of joint
public:
    std::string type;

    /// \brief use force controler
public:
    bool useForce = true;

    /// \brief Control propeller joint.
public:
    std::string jointName;

    /// \brief Control propeller joint.
public:
    physics::JointPtr joint;

    /// \brief direction multiplier for this control
public:
    double multiplier = 1;

    /// \brief input command offset
public:
    double offset = 0;

    /// \brief unused coefficients
public:
    double rotorVelocitySlowdownSim;

public:
    double frequencyCutoff;

public:
    double samplingRate;

public:
    ignition::math::OnePole<double> filter;

public:
    static double kDefaultRotorVelocitySlowdownSim;

public:
    static double kDefaultFrequencyCutoff;

public:
    static double kDefaultSamplingRate;
};

double Control::kDefaultRotorVelocitySlowdownSim = 10.0;
double Control::kDefaultFrequencyCutoff = 5.0;
double Control::kDefaultSamplingRate = 0.2;

// Private data class
class gazebo::ArduPilotPluginPrivate {
    /// \brief Pointer to the update event connection.
public:
    event::ConnectionPtr updateConnection;

    /// \brief Pointer to the model;
public:
    physics::ModelPtr model;

    /// \brief String of the model name;
public:
    std::string modelName;

    /// \brief array of propellers
public:
    // std::vector<Control> controls;
    common::PID pos_controls[3];
    common::PID att_controls[3];

public:
    // send_fdm
    fdmPacket pkt;

    /// \brief keep track of controller update sim-time.
public:
    gazebo::common::Time lastControllerUpdateTime;

    /// \brief Controller update mutex.
public:
    std::mutex mutex;

    /// \brief Pointer to an IMU sensor
public:
    sensors::ImuSensorPtr imuSensor;

    /// \brief Pointer to an GPS sensor
public:
    sensors::GpsSensorPtr gpsSensor;

    /// \brief Pointer to an Rangefinder sensor
public:
    sensors::RaySensorPtr rangefinderSensor;

    /// \brief false before ardupilot controller is online
    /// to allow gazebo to continue without waiting
public:
    bool arduPilotOnline;

    /// \brief number of times ArduCotper skips update
public:
    int connectionTimeoutCount;

    /// \brief number of times ArduCotper skips update
    /// before marking ArduPilot offline
public:
    int connectionTimeoutMaxCount;

public:
    transport::NodePtr node_handle_;

    ros::CallbackQueue rosQueue;

    ros::Publisher motor_pub;

    ros::Publisher servo_pub;

    std::string motor_pub_name;

    std::string servo_pub_name;

    int motor_num;

    int servo_num;

    float motor_speed[MAX_MOTORS];

    float servo_speed[MAX_MOTORS];
};

/////////////////////////////////////////////////
ArduRotorTiltPIDPlugin::ArduRotorTiltPIDPlugin()
    : dataPtr(new ArduPilotPluginPrivate)
{
    this->dataPtr->arduPilotOnline = false;
    this->dataPtr->connectionTimeoutCount = 0;

    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        this->dataPtr->servo_speed[i] = 0.5;
    }
}

/////////////////////////////////////////////////
ArduRotorTiltPIDPlugin::~ArduRotorTiltPIDPlugin() { }

/////////////////////////////////////////////////
void ArduRotorTiltPIDPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "ArduRotorTiltPIDPlugin _model pointer is null");
    GZ_ASSERT(_sdf, "ArduRotorTiltPIDPlugin _sdf pointer is null");

    this->dataPtr->model = _model;
    this->dataPtr->modelName = this->dataPtr->model->GetName();

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, this->dataPtr->modelName + "_plugin", ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle(this->dataPtr->modelName + "_plugin"));

    // this->dataPtr->node_handle_ = transport::NodePtr(new transport::Node());
    // this->dataPtr->node_handle_->Init(this->dataPtr->modelName);

    // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
    // x-forward, y-right, z-down
    // to the aerospace convention: x-forward, y-left, z-up
    this->modelXYZToAirplaneXForwardZDown = ignition::math::Pose3d(0, 0, 0, 0, 0, 0);
    if (_sdf->HasElement("modelXYZToAirplaneXForwardZDown")) {
        this->modelXYZToAirplaneXForwardZDown = _sdf->Get<ignition::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
    }

    // gazeboXYZToNED: from gazebo model frame: x-forward, y-right, z-down
    // to the aerospace convention: x-forward, y-left, z-up
    this->gazeboXYZToNED = ignition::math::Pose3d(0, 0, 0, IGN_PI, 0, 0);
    if (_sdf->HasElement("gazeboXYZToNED")) {
        this->gazeboXYZToNED = _sdf->Get<ignition::math::Pose3d>("gazeboXYZToNED");
    }

    if (_sdf->HasElement("motor_num")) {
        this->dataPtr->motor_num = _sdf->Get<int>("motor_num");
        ROS_INFO_STREAM("motor_num:" << this->dataPtr->motor_num);
    }

    if (_sdf->HasElement("motor_pub")) {
        this->dataPtr->motor_pub_name = _sdf->Get<std::string>("motor_pub");
        ROS_INFO_STREAM("motor_pub_name:" << this->dataPtr->motor_pub_name);
    }

    if (_sdf->HasElement("servo_num")) {
        this->dataPtr->servo_num = _sdf->Get<int>("servo_num");
        ROS_INFO_STREAM("servo_num:" << this->dataPtr->servo_num);
    }

    if (_sdf->HasElement("servo_pub")) {
        this->dataPtr->servo_pub_name = _sdf->Get<std::string>("servo_pub");
        ROS_INFO_STREAM("servo_pub_name:" << this->dataPtr->servo_pub_name);
    }

    this->dataPtr->motor_pub = this->rosNode->advertise<mav_msgs::Actuators>("/" + this->dataPtr->motor_pub_name, 10);

    this->dataPtr->servo_pub = this->rosNode->advertise<mav_msgs::Actuators>("/" + this->dataPtr->servo_pub_name, 10);

    // Get sensors
    std::string imuName = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
    std::vector<std::string> imuScopedName = this->dataPtr->model->SensorScopedName(imuName);

    ROS_INFO_STREAM("imuName:" << imuName);

    if (imuScopedName.size() > 1) {
        gzwarn << "[" << this->dataPtr->modelName << "] "
               << "multiple names match [" << imuName << "] using first found"
               << " name.\n";
        for (unsigned k = 0; k < imuScopedName.size(); ++k) {
            gzwarn << "  sensor " << k << " [" << imuScopedName[k] << "].\n";
        }
    }

    if (imuScopedName.size() > 0) {
        this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(
            sensors::SensorManager::Instance()->GetSensor(imuScopedName[0]));
    }

    if (!this->dataPtr->imuSensor) {
        if (imuScopedName.size() > 1) {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                   << "first imu_sensor scoped name [" << imuScopedName[0]
                   << "] not found, trying the rest of the sensor names.\n";
            for (unsigned k = 1; k < imuScopedName.size(); ++k) {
                this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(
                    sensors::SensorManager::Instance()->GetSensor(imuScopedName[k]));
                if (this->dataPtr->imuSensor) {
                    gzwarn << "found [" << imuScopedName[k] << "]\n";
                    break;
                }
            }
        }

        if (!this->dataPtr->imuSensor) {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                   << "imu_sensor scoped name [" << imuName << "] not found, trying unscoped name.\n"
                   << "\n";
            // TODO: this fails for multi-nested models.
            // TODO: and transforms fail for rotated nested model,
            //       joints point the wrong way.
            this->dataPtr->imuSensor
                = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor(imuName));
        }

        if (!this->dataPtr->imuSensor) {
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "imu_sensor [" << imuName << "] not found, abort ArduPilot plugin.\n"
                  << "\n";
            return;
        }
    }

    ignition::math::Vector3d vector;
    this->dataPtr->pos_controls[0] = common::PID(0, 0, 0, 0, 0, 1, -1);
    if (_sdf->HasElement("pos_control_x")) {
        vector = _sdf->Get<ignition::math::Vector3d>("pos_control_x");
        this->dataPtr->pos_controls[0].SetPGain(vector.X());
        this->dataPtr->pos_controls[0].SetIGain(vector.Y());
        this->dataPtr->pos_controls[0].SetDGain(vector.Z());
        gzwarn << "pos_control_x:[" << this->dataPtr->pos_controls[0].GetPGain() << " "
               << this->dataPtr->pos_controls[0].GetIGain() << " " << this->dataPtr->pos_controls[0].GetDGain() << "] "
               << "\n";
    }

    this->dataPtr->pos_controls[1] = common::PID(0, 0, 0, 0, 0, 1, -1);
    if (_sdf->HasElement("pos_control_y")) {
        vector = _sdf->Get<ignition::math::Vector3d>("pos_control_y");
        this->dataPtr->pos_controls[1].SetPGain(vector.X());
        this->dataPtr->pos_controls[1].SetIGain(vector.Y());
        this->dataPtr->pos_controls[1].SetDGain(vector.Z());
        gzwarn << "pos_control_y:[" << this->dataPtr->pos_controls[1].GetPGain() << " "
               << this->dataPtr->pos_controls[1].GetIGain() << " " << this->dataPtr->pos_controls[1].GetDGain() << "] "
               << "\n";
    }

    this->dataPtr->pos_controls[2] = common::PID(0, 0, 0, 0, 0, 1, -1);
    if (_sdf->HasElement("pos_control_z")) {
        vector = _sdf->Get<ignition::math::Vector3d>("pos_control_z");
        this->dataPtr->pos_controls[2].SetPGain(vector.X());
        this->dataPtr->pos_controls[2].SetIGain(vector.Y());
        this->dataPtr->pos_controls[2].SetDGain(vector.Z());
        gzwarn << "pos_control_z:[" << this->dataPtr->pos_controls[2].GetPGain() << " "
               << this->dataPtr->pos_controls[2].GetIGain() << " " << this->dataPtr->pos_controls[2].GetDGain() << "] "
               << "\n";
    }

    // Controller time control.
    this->dataPtr->lastControllerUpdateTime = 0;

    // Listen to the update event. This event is broadcast every simulation
    // iteration.
    this->dataPtr->updateConnection
        = event::Events::ConnectWorldUpdateBegin(std::bind(&ArduRotorTiltPIDPlugin::OnUpdate, this));

    gzlog << "[" << this->dataPtr->modelName << "] "
          << "ArduPilot ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void ArduRotorTiltPIDPlugin::OnUpdate()
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    const gazebo::common::Time curTime = this->dataPtr->model->GetWorld()->SimTime();

    // Update the control surfaces and publish the new state.
    if (curTime > this->dataPtr->lastControllerUpdateTime) {
        this->SendState();

        this->ApplyMotorForces((curTime - this->dataPtr->lastControllerUpdateTime).Double());
    }

    this->dataPtr->lastControllerUpdateTime = curTime;
}

/////////////////////////////////////////////////
void ArduRotorTiltPIDPlugin::ApplyMotorForces(const double _dt)
{
    mav_msgs::Actuators actuator_msg;
    actuator_msg.angular_velocities.clear();
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[1 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[4 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[2 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[3 - 1]);

    this->dataPtr->motor_pub.publish(actuator_msg);

    for (int i = 0; i < 4; i++) {
        this->dataPtr->servo_speed[i] = (this->dataPtr->servo_speed[i] - 1500) / 500.0f * 180.0f / 57.3f;
    }

    mav_msgs::Actuators servo_msg;
    servo_msg.angular_velocities.clear();
    servo_msg.angular_velocities.push_back(this->dataPtr->servo_speed[1 - 1]);
    servo_msg.angular_velocities.push_back(this->dataPtr->servo_speed[4 - 1]);
    servo_msg.angular_velocities.push_back(this->dataPtr->servo_speed[2 - 1]);
    servo_msg.angular_velocities.push_back(this->dataPtr->servo_speed[3 - 1]);

    this->dataPtr->servo_pub.publish(servo_msg);
}

/////////////////////////////////////////////////
void ArduRotorTiltPIDPlugin::SendState() const
{

    this->dataPtr->pkt.timestamp = this->dataPtr->model->GetWorld()->SimTime().Double();

    // asssumed that the imu orientation is:
    //   x forward
    //   y right
    //   z down

    // get linear acceleration in body frame
    const ignition::math::Vector3d linearAccel = this->dataPtr->imuSensor->LinearAcceleration();

    // copy to pkt
    this->dataPtr->pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
    this->dataPtr->pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
    this->dataPtr->pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
    // gzerr << "lin accel [" << linearAccel << "]\n";

    // get angular velocity in body frame
    const ignition::math::Vector3d angularVel = this->dataPtr->imuSensor->AngularVelocity();

    // copy to pkt
    this->dataPtr->pkt.imuAngularVelocityRPY[0] = angularVel.X();
    this->dataPtr->pkt.imuAngularVelocityRPY[1] = angularVel.Y();
    this->dataPtr->pkt.imuAngularVelocityRPY[2] = angularVel.Z();

    // get inertial pose and velocity
    // position of the uav in world frame
    // this position is used to calcualte bearing and distance
    // from starting location, then use that to update gps position.
    // The algorithm looks something like below (from ardupilot helper
    // libraries):
    //   bearing = to_degrees(atan2(position.y, position.x));
    //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
    //   (self.latitude, self.longitude) = util.gps_newpos(
    //    self.home_latitude, self.home_longitude, bearing, distance)
    // where xyz is in the NED directions.
    // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
    // around.
    // orientation of the uav in world NED frame -
    // assuming the world NED frame has xyz mapped to NED,
    // imuLink is NED - z down

    // model world pose brings us to model,
    // which for example zephyr has -y-forward, x-left, z-up
    // adding modelXYZToAirplaneXForwardZDown rotates
    //   from: model XYZ
    //   to: airplane x-forward, y-left, z-down
    const ignition::math::Pose3d gazeboXYZToModelXForwardZDown
        = this->modelXYZToAirplaneXForwardZDown + this->dataPtr->model->WorldPose();

    // get transform from world NED to Model frame
    const ignition::math::Pose3d NEDToModelXForwardZUp = gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

    // ROS_INFO_STREAM_THROTTLE(1, "ned to model [" << NEDToModelXForwardZUp << "]\n");

    // N
    this->dataPtr->pkt.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

    // E
    this->dataPtr->pkt.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

    // D
    this->dataPtr->pkt.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

    // imuOrientationQuat is the rotation from world NED frame
    // to the uav frame.
    this->dataPtr->pkt.imuOrientationQuat[0] = NEDToModelXForwardZUp.Rot().W();
    this->dataPtr->pkt.imuOrientationQuat[1] = NEDToModelXForwardZUp.Rot().X();
    this->dataPtr->pkt.imuOrientationQuat[2] = NEDToModelXForwardZUp.Rot().Y();
    this->dataPtr->pkt.imuOrientationQuat[3] = NEDToModelXForwardZUp.Rot().Z();

    // gzdbg << "imu [" << gazeboXYZToModelXForwardZDown.rot.GetAsEuler()
    //       << "]\n";
    // gzdbg << "ned [" << this->gazeboXYZToNED.rot.GetAsEuler() << "]\n";
    // gzdbg << "rot [" << NEDToModelXForwardZUp.rot.GetAsEuler() << "]\n";

    // Get NED velocity in body frame *
    // or...
    // Get model velocity in NED frame
    const ignition::math::Vector3d velGazeboWorldFrame = this->dataPtr->model->GetLink()->WorldLinearVel();
    const ignition::math::Vector3d velNEDFrame = this->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
    this->dataPtr->pkt.velocityXYZ[0] = velNEDFrame.X();
    this->dataPtr->pkt.velocityXYZ[1] = velNEDFrame.Y();
    this->dataPtr->pkt.velocityXYZ[2] = velNEDFrame.Z();
}

void ArduRotorTiltPIDPlugin::controller() { }
