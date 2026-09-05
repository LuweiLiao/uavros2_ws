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
#include <array>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <stddef.h>
#ifdef _WIN32
# include <Winsock2.h>
# include <Ws2def.h>
# include <Ws2ipdef.h>
# include <Ws2tcpip.h>
using raw_type = char;
#else
# include <arpa/inet.h>
# include <netinet/in.h>
# include <netinet/tcp.h>
# include <sys/socket.h>
# include <unistd.h>
using raw_type = void;
#endif

#if defined(_MSC_VER)
# include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include "ArduRotorTiltTricopter.hh"
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Filter.hh>
#include <mavlink/v2.0/common/mavlink.h>
#include <mutex>
#include <sdf/sdf.hh>
#include <std_msgs/Float64.h>
#include <string>
#include <vector>

#define MAX_MOTORS 16

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ArduRotorTiltTricopter)

namespace {
constexpr uint8_t kAdm002DeviceAddress = 0x01;
constexpr uint8_t kAdm002EnableStreamFunction = 0x28;
constexpr uint8_t kAdm002WriteOperation = 0x01;
constexpr uint8_t kAdm002EnableStream = 0x01;
constexpr size_t kAdm002EnableFrameLength = 5;
constexpr uint8_t kAdm002StatusPositive = 1U << 0;
constexpr uint8_t kAdm002StatusStable = 1U << 1;
constexpr size_t kAdm002StreamFrameLength = 5;
constexpr double kNewtonToGramForce = 1000.0 / 9.80665;

uint8_t AdditiveChecksum(const uint8_t* _buf, const size_t _len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < _len; ++i) {
        checksum += _buf[i];
    }
    return checksum;
}
}

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
        this->frequencyCutoff          = this->kDefaultFrequencyCutoff;
        this->samplingRate             = this->kDefaultSamplingRate;

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
double Control::kDefaultFrequencyCutoff          = 5.0;
double Control::kDefaultSamplingRate             = 0.2;

// Private data class
class gazebo::ArduPilotSocketPrivate {
    /// \brief constructor
public:
    ArduPilotSocketPrivate()
    {
        // initialize socket udp socket
        fd = socket(AF_INET, SOCK_DGRAM, 0);
#ifndef _WIN32
        // Windows does not support FD_CLOEXEC
        fcntl(fd, F_SETFD, FD_CLOEXEC);
#endif
    }

    /// \brief destructor
public:
    ~ArduPilotSocketPrivate()
    {
        if (fd != -1) {
            ::close(fd);
            fd = -1;
        }
    }

    /// \brief Bind to an adress and port
    /// \param[in] _address Address to bind to.
    /// \param[in] _port Port to bind to.
    /// \return True on success.
public:
    bool Bind(const char* _address, const uint16_t _port)
    {
        struct sockaddr_in sockaddr;
        this->MakeSockAddr(_address, _port, sockaddr);

        if (bind(this->fd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) != 0) {
            shutdown(this->fd, 0);
#ifdef _WIN32
            closesocket(this->fd);
#else
            close(this->fd);
#endif
            return false;
        }
        int one = 1;
        setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));

#ifdef _WIN32
        u_long on = 1;
        ioctlsocket(this->fd, FIONBIO, reinterpret_cast<u_long FAR*>(&on));
#else
        fcntl(this->fd, F_SETFL, fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
#endif
        return true;
    }

    /// \brief Connect to an adress and port
    /// \param[in] _address Address to connect to.
    /// \param[in] _port Port to connect to.
    /// \return True on success.
public:
    bool Connect(const char* _address, const uint16_t _port)
    {
        struct sockaddr_in sockaddr;
        this->MakeSockAddr(_address, _port, sockaddr);

        if (connect(this->fd, (struct sockaddr*)&sockaddr, sizeof(sockaddr)) != 0) {
            shutdown(this->fd, 0);
#ifdef _WIN32
            closesocket(this->fd);
#else
            close(this->fd);
#endif
            return false;
        }
        int one = 1;
        setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));

#ifdef _WIN32
        u_long on = 1;
        ioctlsocket(this->fd, FIONBIO, reinterpret_cast<u_long FAR*>(&on));
#else
        fcntl(this->fd, F_SETFL, fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
#endif
        return true;
    }

    /// \brief Make a socket
    /// \param[in] _address Socket address.
    /// \param[in] _port Socket port
    /// \param[out] _sockaddr New socket address structure.
public:
    void MakeSockAddr(const char* _address, const uint16_t _port, struct sockaddr_in& _sockaddr)
    {
        memset(&_sockaddr, 0, sizeof(_sockaddr));

#ifdef HAVE_SOCK_SIN_LEN
        _sockaddr.sin_len = sizeof(_sockaddr);
#endif

        _sockaddr.sin_port        = htons(_port);
        _sockaddr.sin_family      = AF_INET;
        _sockaddr.sin_addr.s_addr = inet_addr(_address);
    }

public:
    ssize_t Send(const void* _buf, size_t _size) { return send(this->fd, _buf, _size, 0); }

    /// \brief Send data to a UDP peer.
    /// \param[in] _buf Data buffer.
    /// \param[in] _size Size of the buffer.
    /// \param[in] _sockaddr Destination socket address.
    /// \return Bytes sent, or -1 on failure.
public:
    ssize_t SendTo(const void* _buf, size_t _size, const struct sockaddr_in& _sockaddr)
    {
#ifdef _WIN32
        return sendto(this->fd, reinterpret_cast<const char*>(_buf), _size, 0,
            reinterpret_cast<const struct sockaddr*>(&_sockaddr), sizeof(_sockaddr));
#else
        return sendto(this->fd, _buf, _size, 0,
            reinterpret_cast<const struct sockaddr*>(&_sockaddr), sizeof(_sockaddr));
#endif
    }

    /// \brief Receive data
    /// \param[out] _buf Buffer that receives the data.
    /// \param[in] _size Size of the buffer.
    /// \param[in] _timeoutMS Milliseconds to wait for data.
public:
    ssize_t Recv(void* _buf, const size_t _size, uint32_t _timeoutMs)
    {
        fd_set         fds;
        struct timeval tv;

        FD_ZERO(&fds);
        FD_SET(this->fd, &fds);

        tv.tv_sec  = _timeoutMs / 1000;
        tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

        if (select(this->fd + 1, &fds, NULL, NULL, &tv) != 1) {
            return -1;
        }

#ifdef _WIN32
        return recv(this->fd, reinterpret_cast<char*>(_buf), _size, 0);
#else
        return recv(this->fd, _buf, _size, 0);
#endif
    }

    /// \brief Receive one UDP datagram and remember its source address.
    /// \param[out] _buf Buffer that receives the data.
    /// \param[in] _size Size of the buffer.
    /// \param[out] _sockaddr Source socket address.
    /// \param[in] _timeoutMS Milliseconds to wait for data.
public:
    ssize_t RecvFrom(void* _buf, const size_t _size, struct sockaddr_in& _sockaddr, uint32_t _timeoutMs)
    {
        fd_set         fds;
        struct timeval tv;

        FD_ZERO(&fds);
        FD_SET(this->fd, &fds);

        tv.tv_sec  = _timeoutMs / 1000;
        tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

        if (select(this->fd + 1, &fds, NULL, NULL, &tv) != 1) {
            return -1;
        }

        socklen_t sockaddrLen = sizeof(_sockaddr);
#ifdef _WIN32
        return recvfrom(this->fd, reinterpret_cast<char*>(_buf), _size, 0,
            reinterpret_cast<struct sockaddr*>(&_sockaddr), &sockaddrLen);
#else
        return recvfrom(this->fd, _buf, _size, 0,
            reinterpret_cast<struct sockaddr*>(&_sockaddr), &sockaddrLen);
#endif
    }

    /// \brief Socket handle
private:
    int fd;
};

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

    /// \brief Ardupilot Socket for receive motor command on gazebo
public:
    ArduPilotSocketPrivate socket_in;

    /// \brief Ardupilot Socket to send state to Ardupilot
public:
    ArduPilotSocketPrivate socket_out;

    /// \brief ADM002 UDP serial socket.
public:
    ArduPilotSocketPrivate adm002_socket;

    /// \brief Forward-rangefinder MAVLink UDP socket.
public:
    ArduPilotSocketPrivate rangefinder_socket;

    /// \brief Ardupilot address
public:
    std::string fdm_addr;

    /// \brief The Ardupilot listen address
public:
    std::string listen_addr;

    /// \brief Ardupilot port for receiver socket
public:
    uint16_t fdm_port_in;

    /// \brief Ardupilot port for sender socket
public:
    uint16_t fdm_port_out;

    /// \brief Pointer to an IMU sensor
public:
    sensors::ImuSensorPtr imuSensor;

    /// \brief Pointer to an GPS sensor
public:
    sensors::GpsSensorPtr gpsSensor;

    /// \brief Pointer to an Rangefinder sensor
public:
    sensors::RaySensorPtr rangefinderSensor;

    /// \brief Joint force sensor whose reaction force is measured by the ADM002.
public:
    sensors::ForceTorqueSensorPtr frontRodForceTorqueSensor;

    /// \brief Unscoped name used to resolve the force sensor after it is registered.
public:
    std::string frontRodForceTorqueName;

    /// \brief External force along the vehicle body x-axis.
public:
    double frontRodExternalForceBodyX = 0.0;

    /// \brief ADM002 UDP bind address.
public:
    std::string adm002UdpBindAddr = "127.0.0.1";

    /// \brief ADM002 UDP bind port.
public:
    uint16_t adm002UdpBindPort = 9024;

    /// \brief Additional scale applied after converting body-x force to gram-force.
public:
    double adm002ForceScale = 1.0;

    /// \brief True after ArduPilot requests the unsolicited ADM002 stream.
public:
    bool adm002StreamEnabled = false;

    /// \brief UDP peer which requested the ADM002 stream.
public:
    struct sockaddr_in adm002PeerAddr {};

    /// \brief Sim time of the previous ADM002 stream sample.
public:
    gazebo::common::Time adm002LastStreamTime;

    /// \brief ADM002 stream rate.
public:
    double adm002StreamRateHz = 100.0;

    /// \brief UDP bind address for forward rangefinder MAVLink messages.
public:
    std::string rangefinderUdpBindAddr = "127.0.0.1";

    /// \brief UDP bind port for forward rangefinder MAVLink messages.
public:
    uint16_t rangefinderUdpBindPort = 9025;

    /// \brief True after the ArduPilot MAVLink UART sends its first packet.
public:
    bool rangefinderPeerKnown = false;

    /// \brief ArduPilot MAVLink UART peer for rangefinder messages.
public:
    struct sockaddr_in rangefinderPeerAddr {};

    /// \brief Forward rangefinder minimum distance.
public:
    double rangefinderMinDistanceM = 0.05;

    /// \brief Forward rangefinder maximum distance.
public:
    double rangefinderMaxDistanceM = 10.0;

    /// \brief Forward rangefinder update rate.
public:
    double rangefinderRateHz = 50.0;

    /// \brief Sim time of the previous rangefinder message.
public:
    gazebo::common::Time rangefinderLastSendTime;

    /// \brief MAVLink channel sequence state for the rangefinder stream.
public:
    mavlink_status_t rangefinderMavlinkStatus {};

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
    std::string    motor_pub_name;

    ros::Publisher servo1_pub;
    std::string    servo1_pub_name;

    ros::Publisher servo2_pub;
    std::string    servo2_pub_name;

    ros::Publisher front_rod_contact_force_x_pub;
    std::string    front_rod_contact_force_x_topic;

    int motor_num;

    int servo_num;

    float motor_speed[MAX_MOTORS];
    float servo1_speed[MAX_MOTORS];
    // float servo2_speed[MAX_MOTORS];
};

/////////////////////////////////////////////////
ArduRotorTiltTricopter::ArduRotorTiltTricopter()
    : dataPtr(new ArduPilotPluginPrivate)
{
    this->dataPtr->arduPilotOnline        = false;
    this->dataPtr->connectionTimeoutCount = 0;

    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        this->dataPtr->servo1_speed[i] = 0.5;
    }

    // for (uint8_t i = 0; i < MAX_MOTORS; i++) {
    //     this->dataPtr->servo2_speed[i] = 0.5;
    // }
}

/////////////////////////////////////////////////
ArduRotorTiltTricopter::~ArduRotorTiltTricopter() { }

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "ArduRotorTiltTricopter _model pointer is null");
    GZ_ASSERT(_sdf, "ArduRotorTiltTricopter _sdf pointer is null");

    this->dataPtr->model     = _model;
    this->dataPtr->modelName = this->dataPtr->model->GetName();

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
        int    argc = 0;
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

    if (_sdf->HasElement("servo1_pub")) {
        this->dataPtr->servo1_pub_name = _sdf->Get<std::string>("servo1_pub");
        ROS_INFO_STREAM("servo1_pub_name:" << this->dataPtr->servo1_pub_name);
    }

    // if (_sdf->HasElement("servo2_pub")) {
    //     this->dataPtr->servo2_pub_name = _sdf->Get<std::string>("servo2_pub");
    //     ROS_INFO_STREAM("servo2_pub_name:" << this->dataPtr->servo2_pub_name);
    // }

    this->dataPtr->motor_pub  = this->rosNode->advertise<mav_msgs::Actuators>("/" + this->dataPtr->motor_pub_name, 10);
    this->dataPtr->servo1_pub = this->rosNode->advertise<mav_msgs::Actuators>("/" + this->dataPtr->servo1_pub_name, 10);
    // this->dataPtr->servo2_pub = this->rosNode->advertise<mav_msgs::Actuators>("/" + this->dataPtr->servo2_pub_name, 10);

    this->dataPtr->front_rod_contact_force_x_topic =
        _sdf->Get("frontRodContactForceXTopic", static_cast<std::string>("/tilt_tricopter/front_rod/contact_force_x")).first;
    this->dataPtr->front_rod_contact_force_x_pub =
        this->rosNode->advertise<std_msgs::Float64>(this->dataPtr->front_rod_contact_force_x_topic, 10);

    this->dataPtr->frontRodForceTorqueName =
        _sdf->Get("frontRodForceTorqueName", static_cast<std::string>("front_rod_force_torque")).first;
    const std::vector<std::string> frontRodForceTorqueScopedNames =
        this->dataPtr->model->SensorScopedName(this->dataPtr->frontRodForceTorqueName);
    for (const std::string& scopedName : frontRodForceTorqueScopedNames) {
        this->dataPtr->frontRodForceTorqueSensor =
            std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(
                sensors::SensorManager::Instance()->GetSensor(scopedName));
        if (this->dataPtr->frontRodForceTorqueSensor) {
            break;
        }
    }
    if (this->dataPtr->frontRodForceTorqueSensor) {
        ROS_INFO_STREAM("front_rod force sensor:"
                        << this->dataPtr->frontRodForceTorqueSensor->ScopedName());
    } else {
        ROS_INFO_STREAM("front_rod force sensor ["
                        << this->dataPtr->frontRodForceTorqueName
                        << "] will be resolved after Gazebo registers sensors");
    }

    this->dataPtr->adm002UdpBindAddr =
        _sdf->Get("adm002UdpBindAddr", static_cast<std::string>("127.0.0.1")).first;
    this->dataPtr->adm002UdpBindPort =
        _sdf->Get("adm002UdpBindPort", static_cast<uint32_t>(9024)).first;
    this->dataPtr->adm002ForceScale =
        _sdf->Get("adm002ForceScale", 1.0).first;
    this->dataPtr->adm002StreamRateHz =
        _sdf->Get("adm002StreamRateHz", 100.0).first;
    if (!this->dataPtr->adm002_socket.Bind(
            this->dataPtr->adm002UdpBindAddr.c_str(), this->dataPtr->adm002UdpBindPort)) {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "failed to bind ADM002 UDP socket with "
              << this->dataPtr->adm002UdpBindAddr << ":" << this->dataPtr->adm002UdpBindPort
              << " aborting plugin.\n";
        return;
    }
    ROS_INFO_STREAM("ADM002 UDP serial:" << this->dataPtr->adm002UdpBindAddr
                    << ":" << this->dataPtr->adm002UdpBindPort
                    << " force_scale:" << this->dataPtr->adm002ForceScale
                    << " stream_rate_hz:" << this->dataPtr->adm002StreamRateHz);

    this->dataPtr->rangefinderUdpBindAddr =
        _sdf->Get("rangefinderUdpBindAddr", static_cast<std::string>("127.0.0.1")).first;
    this->dataPtr->rangefinderUdpBindPort =
        _sdf->Get("rangefinderUdpBindPort", static_cast<uint32_t>(9025)).first;
    this->dataPtr->rangefinderMinDistanceM =
        _sdf->Get("rangefinderMinDistanceM", 0.05).first;
    this->dataPtr->rangefinderMaxDistanceM =
        _sdf->Get("rangefinderMaxDistanceM", 10.0).first;
    this->dataPtr->rangefinderRateHz =
        _sdf->Get("rangefinderRateHz", 50.0).first;
    if (!this->dataPtr->rangefinder_socket.Bind(
            this->dataPtr->rangefinderUdpBindAddr.c_str(), this->dataPtr->rangefinderUdpBindPort)) {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "failed to bind forward rangefinder UDP socket to "
              << this->dataPtr->rangefinderUdpBindAddr << ":" << this->dataPtr->rangefinderUdpBindPort
              << " aborting plugin.\n";
        return;
    }

    const std::string rangefinderName =
        _sdf->Get("rangefinderName", static_cast<std::string>("front_rangefinder")).first;
    const std::vector<std::string> rangefinderScopedNames =
        this->dataPtr->model->SensorScopedName(rangefinderName);
    for (const std::string& scopedName : rangefinderScopedNames) {
        this->dataPtr->rangefinderSensor = std::dynamic_pointer_cast<sensors::RaySensor>(
            sensors::SensorManager::Instance()->GetSensor(scopedName));
        if (this->dataPtr->rangefinderSensor) {
            break;
        }
    }
    if (!this->dataPtr->rangefinderSensor) {
        this->dataPtr->rangefinderSensor = std::dynamic_pointer_cast<sensors::RaySensor>(
            sensors::SensorManager::Instance()->GetSensor(rangefinderName));
    }
    if (!this->dataPtr->rangefinderSensor) {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "forward rangefinder [" << rangefinderName << "] not found, aborting plugin.\n";
        return;
    }
    ROS_INFO_STREAM("Forward rangefinder:" << this->dataPtr->rangefinderSensor->ScopedName()
                    << " UDP:" << this->dataPtr->rangefinderUdpBindAddr
                    << ":" << this->dataPtr->rangefinderUdpBindPort);

    // Get sensors
    std::string              imuName       = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
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
            this->dataPtr->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>(sensors::SensorManager::Instance()->GetSensor(imuName));
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

    // Initialise ardupilot sockets
    if (!InitArduPilotSockets(_sdf)) {
        return;
    }

    // Missed update count before we declare arduPilotOnline status false
    this->dataPtr->connectionTimeoutMaxCount = _sdf->Get("connectionTimeoutMaxCount", 10).first;

    // Listen to the update event. This event is broadcast every simulation
    // iteration.
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ArduRotorTiltTricopter::OnUpdate, this));

    gzlog << "[" << this->dataPtr->modelName << "] "
          << "ArduPilot ready to fly. The force will be with you" << std::endl;
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::OnUpdate()
{
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

    const gazebo::common::Time curTime = this->dataPtr->model->GetWorld()->SimTime();

    // Update the control surfaces and publish the new state.
    if (curTime > this->dataPtr->lastControllerUpdateTime) {
        this->UpdateFrontRodContactWrench();
        this->ProcessAdm002Udp();
        this->SendForwardRangefinder();
        this->ReceiveMotorCommand();
        if (this->dataPtr->arduPilotOnline) {
            this->ApplyMotorForces((curTime - this->dataPtr->lastControllerUpdateTime).Double());
            this->SendState();
        }
    }

    this->dataPtr->lastControllerUpdateTime = curTime;
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::UpdateFrontRodContactWrench()
{
    this->dataPtr->frontRodExternalForceBodyX = 0.0;

    if (!this->dataPtr->frontRodForceTorqueSensor) {
        const sensors::Sensor_V sensors = sensors::SensorManager::Instance()->GetSensors();
        for (const sensors::SensorPtr& sensor : sensors) {
            if (!sensor || sensor->Name() != this->dataPtr->frontRodForceTorqueName) {
                continue;
            }
            this->dataPtr->frontRodForceTorqueSensor =
                std::dynamic_pointer_cast<sensors::ForceTorqueSensor>(sensor);
            if (this->dataPtr->frontRodForceTorqueSensor) {
                ROS_INFO_STREAM("front_rod force sensor:"
                                << this->dataPtr->frontRodForceTorqueSensor->ScopedName());
                break;
            }
        }
        if (!this->dataPtr->frontRodForceTorqueSensor) {
            return;
        }
    }

    this->dataPtr->frontRodExternalForceBodyX =
        this->dataPtr->frontRodForceTorqueSensor->Force().X();

    if (this->dataPtr->front_rod_contact_force_x_pub) {
        std_msgs::Float64 msg;
        msg.data = this->dataPtr->frontRodExternalForceBodyX;
        this->dataPtr->front_rod_contact_force_x_pub.publish(msg);
    }
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::ProcessAdm002Udp()
{
    std::array<uint8_t, 256> request;
    struct sockaddr_in peerAddr {};

    while (true) {
        const ssize_t recvSize = this->dataPtr->adm002_socket.RecvFrom(
            request.data(), request.size(), peerAddr, 0ul);
        if (recvSize == -1) {
            break;
        }

        if (recvSize == static_cast<ssize_t>(kAdm002EnableFrameLength) &&
            request[0] == kAdm002DeviceAddress &&
            request[1] == kAdm002EnableStreamFunction &&
            request[2] == kAdm002WriteOperation &&
            request[3] == kAdm002EnableStream &&
            request[4] == AdditiveChecksum(request.data(), kAdm002EnableFrameLength - 1)) {
            this->dataPtr->adm002PeerAddr = peerAddr;
            this->dataPtr->adm002StreamEnabled = true;
        }
    }

    if (!this->dataPtr->adm002StreamEnabled) {
        return;
    }

    const gazebo::common::Time now = this->dataPtr->model->GetWorld()->SimTime();
    const double streamPeriodS = 1.0 / std::max(this->dataPtr->adm002StreamRateHz, 1.0);
    if ((now - this->dataPtr->adm002LastStreamTime).Double() < streamPeriodS) {
        return;
    }
    this->dataPtr->adm002LastStreamTime = now;

    double contactForceGram = this->dataPtr->frontRodExternalForceBodyX *
        kNewtonToGramForce * this->dataPtr->adm002ForceScale;
    if (!std::isfinite(contactForceGram)) {
        contactForceGram = 0.0;
    }
    contactForceGram = ignition::math::clamp(contactForceGram, -16777215.0, 16777215.0);
    const int32_t signedWeightG = static_cast<int32_t>(std::llround(contactForceGram));
    const uint32_t magnitudeG = static_cast<uint32_t>(std::abs(signedWeightG));

    uint8_t frame[kAdm002StreamFrameLength] {
        static_cast<uint8_t>(kAdm002StatusStable |
            (signedWeightG >= 0 ? kAdm002StatusPositive : 0)),
        static_cast<uint8_t>(magnitudeG >> 16),
        static_cast<uint8_t>(magnitudeG >> 8),
        static_cast<uint8_t>(magnitudeG),
        0,
    };
    frame[kAdm002StreamFrameLength - 1] =
        AdditiveChecksum(frame, kAdm002StreamFrameLength - 1);
    this->dataPtr->adm002_socket.SendTo(
        frame, sizeof(frame), this->dataPtr->adm002PeerAddr);
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::SendForwardRangefinder()
{
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> request;
    struct sockaddr_in peerAddr {};
    while (true) {
        const ssize_t recvSize = this->dataPtr->rangefinder_socket.RecvFrom(
            request.data(), request.size(), peerAddr, 0ul);
        if (recvSize == -1) {
            break;
        }
        this->dataPtr->rangefinderPeerAddr = peerAddr;
        this->dataPtr->rangefinderPeerKnown = true;
    }

    if (!this->dataPtr->rangefinderPeerKnown) {
        return;
    }

    if (!this->dataPtr->rangefinderSensor) {
        return;
    }

    const gazebo::common::Time now = this->dataPtr->model->GetWorld()->SimTime();
    const double sendPeriodS = 1.0 / std::max(this->dataPtr->rangefinderRateHz, 1.0);
    if ((now - this->dataPtr->rangefinderLastSendTime).Double() < sendPeriodS) {
        return;
    }
    this->dataPtr->rangefinderLastSendTime = now;

    const double rangeM = this->dataPtr->rangefinderSensor->Range(0);
    const bool validRange = std::isfinite(rangeM) &&
        rangeM >= this->dataPtr->rangefinderMinDistanceM &&
        rangeM <= this->dataPtr->rangefinderMaxDistanceM;
    const uint16_t minDistanceCm = static_cast<uint16_t>(std::llround(
        this->dataPtr->rangefinderMinDistanceM * 100.0));
    const uint16_t maxDistanceCm = static_cast<uint16_t>(std::llround(
        this->dataPtr->rangefinderMaxDistanceM * 100.0));
    const uint16_t currentDistanceCm = validRange ?
        static_cast<uint16_t>(std::llround(rangeM * 100.0)) : maxDistanceCm + 1U;

    mavlink_distance_sensor_t sensorMsg {};
    sensorMsg.time_boot_ms = static_cast<uint32_t>(now.Double() * 1000.0);
    sensorMsg.min_distance = minDistanceCm;
    sensorMsg.max_distance = maxDistanceCm;
    sensorMsg.current_distance = currentDistanceCm;
    sensorMsg.type = MAV_DISTANCE_SENSOR_LASER;
    sensorMsg.id = 0;
    sensorMsg.orientation = MAV_SENSOR_ROTATION_NONE;
    sensorMsg.covariance = UINT8_MAX;
    sensorMsg.signal_quality = validRange ? 100U : 0U;

    mavlink_message_t msg;
    mavlink_msg_distance_sensor_encode_status(
        32, MAV_COMP_ID_PERIPHERAL, &this->dataPtr->rangefinderMavlinkStatus, &msg, &sensorMsg);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    this->dataPtr->rangefinder_socket.SendTo(
        buffer, len, this->dataPtr->rangefinderPeerAddr);
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::ResetPIDs()
{
    // Reset velocity PID for controls
    for (size_t i = 0; i < this->dataPtr->controls.size(); ++i) {
        this->dataPtr->controls[i].cmd = 0;
        // this->dataPtr->controls[i].pid.Reset();
    }
}

/////////////////////////////////////////////////
bool ArduRotorTiltTricopter::InitArduPilotSockets(sdf::ElementPtr _sdf) const
{
    this->dataPtr->fdm_addr     = _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;
    this->dataPtr->listen_addr  = _sdf->Get("listen_addr", static_cast<std::string>("127.0.0.1")).first;
    this->dataPtr->fdm_port_in  = _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;
    this->dataPtr->fdm_port_out = _sdf->Get("fdm_port_out", static_cast<uint32_t>(9003)).first;

    if (!this->dataPtr->socket_in.Bind(this->dataPtr->listen_addr.c_str(), this->dataPtr->fdm_port_in)) {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "failed to bind with " << this->dataPtr->listen_addr << ":" << this->dataPtr->fdm_port_in
              << " aborting plugin.\n";
        return false;
    }

    if (!this->dataPtr->socket_out.Connect(this->dataPtr->fdm_addr.c_str(), this->dataPtr->fdm_port_out)) {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "failed to bind with " << this->dataPtr->fdm_addr << ":" << this->dataPtr->fdm_port_out
              << " aborting plugin.\n";
        return false;
    }

    return true;
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::ApplyMotorForces(const double _dt)
{
    mav_msgs::Actuators actuator_msg;
    actuator_msg.angular_velocities.clear();
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[1 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[2 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[3 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[4 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[5 - 1]);
    actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[6 - 1]);
    this->dataPtr->motor_pub.publish(actuator_msg);

    for (int i = 0; i < this->dataPtr->servo_num; i++) {
        this->dataPtr->servo1_speed[i] = (this->dataPtr->servo1_speed[i] - 1500) / 1000.0f * 135.0f / 57.3f;
    }
    mav_msgs::Actuators servo_msg;
    servo_msg.angular_velocities.clear();
    servo_msg.angular_velocities.push_back(this->dataPtr->servo1_speed[1 - 1]);
    servo_msg.angular_velocities.push_back(this->dataPtr->servo1_speed[2 - 1]);
    servo_msg.angular_velocities.push_back(this->dataPtr->servo1_speed[3 - 1]);
    this->dataPtr->servo1_pub.publish(servo_msg);
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::ReceiveMotorCommand()
{
    // Added detection for whether ArduPilot is online or not.
    // If ArduPilot is detected (receive of fdm packet from someone),
    // then socket receive wait time is increased from 1ms to 1 sec
    // to accomodate network jitter.
    // If ArduPilot is not detected, receive call blocks for 1ms
    // on each call.
    // Once ArduPilot presence is detected, it takes this many
    // missed receives before declaring the FCS offline.

    ServoPacket pkt;
    uint32_t    waitMs;
    if (this->dataPtr->arduPilotOnline) {
        // increase timeout for receive once we detect a packet from
        // ArduPilot FCS.
        waitMs = 1000;
    } else {
        // Otherwise skip quickly and do not set control force.
        waitMs = 1;
    }
    ssize_t recvSize = this->dataPtr->socket_in.Recv(&pkt, sizeof(ServoPacket), waitMs);

    // Drain the socket in the case we're backed up
    int         counter = 0;
    ServoPacket last_pkt;
    while (true) {
        // last_pkt = pkt;
        const ssize_t recvSize_last = this->dataPtr->socket_in.Recv(&last_pkt, sizeof(ServoPacket), 0ul);
        if (recvSize_last == -1) {
            break;
        }
        counter++;
        pkt      = last_pkt;
        recvSize = recvSize_last;
    }
    if (counter > 0) {
        gzdbg << "[" << this->dataPtr->modelName << "] "
              << "Drained n packets: " << counter << std::endl;
    }

    if (recvSize == -1) {
        // didn't receive a packet
        // gzdbg << "no packet\n";
        gazebo::common::Time::NSleep(100);
        if (this->dataPtr->arduPilotOnline) {
            gzwarn << "[" << this->dataPtr->modelName << "] "
                   << "Broken ArduPilot connection, count [" << this->dataPtr->connectionTimeoutCount << "/"
                   << this->dataPtr->connectionTimeoutMaxCount << "]\n";
            if (++this->dataPtr->connectionTimeoutCount > this->dataPtr->connectionTimeoutMaxCount) {
                this->dataPtr->connectionTimeoutCount = 0;
                this->dataPtr->arduPilotOnline        = false;
                gzwarn << "[" << this->dataPtr->modelName << "] "
                       << "Broken ArduPilot connection, resetting motor control.\n";
                this->ResetPIDs();
            }
        }
    } else {
        const ssize_t expectedPktSize = sizeof(pkt.motorSpeed[0]) * (this->dataPtr->motor_num + this->dataPtr->servo_num);
        if (recvSize < expectedPktSize) {
            gzerr << "[" << this->dataPtr->modelName << "] "
                  << "got less than model needs. Got: " << recvSize << "commands, expected size: " << expectedPktSize
                  << "\n";
        }
        const ssize_t recvChannels = recvSize / sizeof(pkt.motorSpeed[0]);

        if (!this->dataPtr->arduPilotOnline) {
            gzdbg << "[" << this->dataPtr->modelName << "] "
                  << "ArduPilot controller online detected.\n";
            // made connection, set some flags
            this->dataPtr->connectionTimeoutCount = 0;
            this->dataPtr->arduPilotOnline        = true;
        }

        // compute command based on requested motorSpeed
        // std::cout << "motorSpeed:\t";
        for (unsigned i = 0; i < this->dataPtr->motor_num; ++i) {
            if (i < MAX_MOTORS) {
                const double cmd              = ignition::math::clamp(pkt.motorSpeed[i], 0.0f, 1.0f);
                this->dataPtr->motor_speed[i] = cmd * 1000.0f;

                // std::cout << cmd << "\t";
            } else {
                gzerr << "[" << this->dataPtr->modelName << "] "
                      << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
            }
        }

        for (unsigned i = 0; i < this->dataPtr->servo_num; ++i) {
            if (i < MAX_MOTORS) {
                double cmd                     = ignition::math::clamp(pkt.motorSpeed[i + this->dataPtr->motor_num], -2.0f, 2.0f);
                this->dataPtr->servo1_speed[i] = (cmd - 0.5f) * 1000.0f + 1500.0f;
            } else {
                gzerr << "[" << this->dataPtr->modelName << "] "
                      << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
            }
        }
    }
}

/////////////////////////////////////////////////
void ArduRotorTiltTricopter::SendState() const
{
    // send_fdm
    fdmPacket pkt;

    pkt.timestamp = this->dataPtr->model->GetWorld()->SimTime().Double();

    // asssumed that the imu orientation is:
    //   x forward
    //   y right
    //   z down

    // get linear acceleration in body frame
    const ignition::math::Vector3d linearAccel = this->dataPtr->imuSensor->LinearAcceleration();

    // copy to pkt
    pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
    pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
    pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
    // gzerr << "lin accel [" << linearAccel << "]\n";

    // get angular velocity in body frame
    const ignition::math::Vector3d angularVel = this->dataPtr->imuSensor->AngularVelocity();

    // copy to pkt
    pkt.imuAngularVelocityRPY[0] = angularVel.X();
    pkt.imuAngularVelocityRPY[1] = angularVel.Y();
    pkt.imuAngularVelocityRPY[2] = angularVel.Z();

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
    const ignition::math::Pose3d gazeboXYZToModelXForwardZDown = this->modelXYZToAirplaneXForwardZDown + this->dataPtr->model->WorldPose();

    // get transform from world NED to Model frame
    const ignition::math::Pose3d NEDToModelXForwardZUp = gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

    // ROS_INFO_STREAM_THROTTLE(1, "ned to model [" << NEDToModelXForwardZUp << "]\n");

    // N
    pkt.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

    // E
    pkt.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

    // D
    pkt.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

    // imuOrientationQuat is the rotation from world NED frame
    // to the uav frame.
    pkt.imuOrientationQuat[0] = NEDToModelXForwardZUp.Rot().W();
    pkt.imuOrientationQuat[1] = NEDToModelXForwardZUp.Rot().X();
    pkt.imuOrientationQuat[2] = NEDToModelXForwardZUp.Rot().Y();
    pkt.imuOrientationQuat[3] = NEDToModelXForwardZUp.Rot().Z();

    // gzdbg << "imu [" << gazeboXYZToModelXForwardZDown.rot.GetAsEuler()
    //       << "]\n";
    // gzdbg << "ned [" << this->gazeboXYZToNED.rot.GetAsEuler() << "]\n";
    // gzdbg << "rot [" << NEDToModelXForwardZUp.rot.GetAsEuler() << "]\n";

    // Get NED velocity in body frame *
    // or...
    // Get model velocity in NED frame
    const ignition::math::Vector3d velGazeboWorldFrame = this->dataPtr->model->GetLink()->WorldLinearVel();
    const ignition::math::Vector3d velNEDFrame         = this->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
    pkt.velocityXYZ[0]                                 = velNEDFrame.X();
    pkt.velocityXYZ[1]                                 = velNEDFrame.Y();
    pkt.velocityXYZ[2]                                 = velNEDFrame.Z();
    /* NOT MERGED IN MASTER YET
  if (!this->dataPtr->gpsSensor)
    {

    }
    else {
        pkt.longitude = this->dataPtr->gpsSensor->Longitude().Degree();
        pkt.latitude = this->dataPtr->gpsSensor->Latitude().Degree();
        pkt.altitude = this->dataPtr->gpsSensor->Altitude();
    }

    // TODO : make generic enough to accept sonar/gpuray etc. too
    if (!this->dataPtr->rangefinderSensor)
    {

    } else {
        // Rangefinder value can not be send as Inf to ardupilot
        const double range = this->dataPtr->rangefinderSensor->Range(0);
        pkt.rangefinder = std::isinf(range) ? 0.0 : range;
    }

  // airspeed :     wind = Vector3(environment.wind.x, environment.wind.y, environment.wind.z)
   // pkt.airspeed = (pkt.velocity - wind).length()
*/
    this->dataPtr->socket_out.Send(&pkt, sizeof(pkt));
}
