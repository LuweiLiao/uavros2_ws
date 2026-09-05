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

#include "MyBalancebotPlugin.hh"
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

GZ_REGISTER_MODEL_PLUGIN(MyBalancebotPlugin)

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

    double wheel_v[2];
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
    std::vector<Control> controls;

    /// \brief keep track of controller update sim-time.
public:
    gazebo::common::Time lastControllerUpdateTime;

    /// \brief Controller update mutex.
public:
    std::mutex mutex;

    /// \brief Pointer to an IMU sensor
public:
    sensors::ImuSensorPtr imuSensor;

public:
    transport::NodePtr node_handle_;

    ros::CallbackQueue rosQueue;

    ros::Publisher motor_pub;

    std::string motor_pub_name;

    int motor_num;

    float motor_speed[MAX_MOTORS];

    physics::LinkPtr wheel_1_link;
    physics::LinkPtr wheel_2_link;

public:
    fdmPacket pkt;

};

/////////////////////////////////////////////////
MyBalancebotPlugin::MyBalancebotPlugin()
    : dataPtr(new ArduPilotPluginPrivate)
{
}

/////////////////////////////////////////////////
MyBalancebotPlugin::~MyBalancebotPlugin() { }

/////////////////////////////////////////////////
void MyBalancebotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "MyBalancebotPlugin _model pointer is null");
    GZ_ASSERT(_sdf, "MyBalancebotPlugin _sdf pointer is null");

    this->dataPtr->model = _model;
    this->dataPtr->modelName = this->dataPtr->model->GetName();

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc, argv, this->dataPtr->modelName + "_plugin", ros::init_options::NoSigintHandler);
    }
    this->rosNode.reset(new ros::NodeHandle(this->dataPtr->modelName + "_plugin"));

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

    this->dataPtr->motor_pub = this->rosNode->advertise<mav_msgs::Actuators>("/" + this->dataPtr->motor_pub_name, 10);

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

    // Controller time control.
    this->dataPtr->lastControllerUpdateTime = 0;

    // Listen to the update event. This event is broadcast every simulation
    // iteration.
    this->dataPtr->updateConnection
        = event::Events::ConnectWorldUpdateBegin(std::bind(&MyBalancebotPlugin::OnUpdate, this));

    // gzlog << "[" << this->dataPtr->modelName << "] "
    //       << "ArduPilot ready to fly. The force will be with you" << std::endl;

    std::string link_name_;
    if (_sdf->HasElement("wheel_1_name")) {
        link_name_ = _sdf->GetElement("wheel_1_name")->Get<std::string>();

        this->dataPtr->wheel_1_link = this->dataPtr->model->GetLink(link_name_);

        if (this->dataPtr->wheel_1_link == NULL)
            gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \""
                    << link_name_ << "\".");
    }
    if (_sdf->HasElement("wheel_2_name")) {
        link_name_ = _sdf->GetElement("wheel_2_name")->Get<std::string>();

        this->dataPtr->wheel_2_link = this->dataPtr->model->GetLink(link_name_);

        if (this->dataPtr->wheel_2_link == NULL)
            gzthrow("[gazebo_odometry_plugin] Couldn't find specified link \""
                    << link_name_ << "\".");
    }
}

/////////////////////////////////////////////////
void MyBalancebotPlugin::OnUpdate()
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    const gazebo::common::Time curTime = this->dataPtr->model->GetWorld()->SimTime();

    // Update the control surfaces and publish the new state.
    if (curTime > this->dataPtr->lastControllerUpdateTime) {
        this->ApplyMotorForces((curTime - this->dataPtr->lastControllerUpdateTime).Double());
        this->SendState();
    }

    this->dataPtr->lastControllerUpdateTime = curTime;
}

/////////////////////////////////////////////////
void MyBalancebotPlugin::ResetPIDs()
{
    // Reset velocity PID for controls
    for (size_t i = 0; i < this->dataPtr->controls.size(); ++i) {
        this->dataPtr->controls[i].cmd = 0;
        // this->dataPtr->controls[i].pid.Reset();
    }
}

/////////////////////////////////////////////////
void MyBalancebotPlugin::ApplyMotorForces(const double _dt)
{
    mav_msgs::Actuators actuator_msg;
    actuator_msg.angular_velocities.clear();

    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[5 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[6 - 1]);
    this->dataPtr->motor_pub.publish(actuator_msg);
}

/////////////////////////////////////////////////
void MyBalancebotPlugin::SendState() const
{


    this->dataPtr->pkt.timestamp = this->dataPtr->model->GetWorld()->SimTime().Double();

    // asssumed that the imu orientation is:
    //   x forward
    //   y right
    //   z down

    // get linear acceleration in body frame
    const ignition::math::Vector3d linearAccel = this->dataPtr->imuSensor->LinearAcceleration();

    // copy to this->dataPtr->pkt
    this->dataPtr->pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
    this->dataPtr->pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
    this->dataPtr->pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
    // gzerr << "lin accel [" << linearAccel << "]\n";

    // get angular velocity in body frame
    const ignition::math::Vector3d angularVel = this->dataPtr->imuSensor->AngularVelocity();

    // copy to this->dataPtr->pkt
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

    ignition::math::Vector3d wheel_1_v = this->dataPtr->wheel_1_link->RelativeAngularVel();
    ignition::math::Vector3d wheel_2_v = this->dataPtr->wheel_2_link->RelativeAngularVel();

    this->dataPtr->pkt.wheel_v[0] = wheel_1_v.Z();
    this->dataPtr->pkt.wheel_v[1] = wheel_2_v.Z();

}
