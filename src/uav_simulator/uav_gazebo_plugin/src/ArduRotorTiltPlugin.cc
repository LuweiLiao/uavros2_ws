/*
 * Gazebo Sim / ROS 2 port of the ROS 1 ArduRotorTiltPlugin.
 *
 * This plugin is the transport/output variant used by the original USL IWP
 * models.  It does not replace the RotorS motor model: it receives the same
 * ArduPilot 16-float UDP servo packet, publishes the original motor and tilt
 * actuator messages, and sends the original 17-double FDM packet back to
 * SITL.  The public SDF and topic contracts are intentionally unchanged.
 */

#include "ArduRotorTiltPlugin.hh"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gz/math/Pose3.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <mav_msgs/msg/actuators.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

constexpr std::size_t kMaxMotors = 16;

struct ServoPacket
{
  float motorSpeed[kMaxMotors];
};

struct FdmPacket
{
  double timestamp;
  double imuAngularVelocityRPY[3];
  double imuLinearAccelerationXYZ[3];
  double imuOrientationQuat[4];
  double velocityXYZ[3];
  double positionXYZ[3];
};

static_assert(sizeof(ServoPacket) == 16u * sizeof(float),
              "Unexpected ArduPilot servo packet padding");
static_assert(sizeof(FdmPacket) == 17u * sizeof(double),
              "Unexpected ArduPilot FDM packet padding");

class UdpSocket
{
public:
  UdpSocket() = default;
  ~UdpSocket()
  {
    Close();
  }

  UdpSocket(const UdpSocket &) = delete;
  UdpSocket &operator=(const UdpSocket &) = delete;

  bool Open()
  {
    if (fd_ >= 0)
      return true;
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0)
      return false;
    const int flags = ::fcntl(fd_, F_GETFD, 0);
    if (flags >= 0)
      (void)::fcntl(fd_, F_SETFD, flags | FD_CLOEXEC);
    const int one = 1;
    (void)::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    const int status = ::fcntl(fd_, F_GETFL, 0);
    if (status >= 0)
      (void)::fcntl(fd_, F_SETFL, status | O_NONBLOCK);
    return true;
  }

  bool Bind(const std::string &_address, uint16_t _port)
  {
    if (!Open())
      return false;
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(_port);
    if (::inet_pton(AF_INET, _address.c_str(), &endpoint.sin_addr) != 1)
      return false;
    return ::bind(fd_, reinterpret_cast<const sockaddr *>(&endpoint),
                  sizeof(endpoint)) == 0;
  }

  bool Connect(const std::string &_address, uint16_t _port)
  {
    if (!Open())
      return false;
    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(_port);
    if (::inet_pton(AF_INET, _address.c_str(), &endpoint.sin_addr) != 1)
      return false;
    return ::connect(fd_, reinterpret_cast<const sockaddr *>(&endpoint),
                     sizeof(endpoint)) == 0;
  }

  ssize_t Receive(void *_buffer, std::size_t _size)
  {
    if (fd_ < 0)
      return -1;
    const ssize_t result = ::recv(fd_, _buffer, _size, 0);
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      return -1;
    return result;
  }

  ssize_t Send(const void *_buffer, std::size_t _size) const
  {
    if (fd_ < 0)
      return -1;
    return ::send(fd_, _buffer, _size, 0);
  }

private:
  void Close()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  int fd_{-1};
};

std::string NormalizeTopic(std::string _topic)
{
  while (_topic.size() > 1 && _topic.back() == '/')
    _topic.pop_back();
  if (_topic.empty())
    return _topic;
  if (_topic.front() != '/')
    _topic.insert(_topic.begin(), '/');
  return _topic;
}

std::string LastScopePart(const std::string &_name)
{
  const std::size_t separator = _name.rfind("::");
  if (separator == std::string::npos)
    return _name;
  return _name.substr(separator + 2);
}

}  // namespace

namespace gazebo
{

class ArduRotorTiltPlugin::Private
{
public:
  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  gz::sim::Entity baseLink{gz::sim::kNullEntity};
  gz::sim::Entity imuLink{gz::sim::kNullEntity};
  gz::sim::Entity imuSensor{gz::sim::kNullEntity};

  std::string modelName;
  std::string imuName{"imu_sensor"};
  std::string motorTopic{"/gazebo/command/prop_speed"};
  std::string servoTopic{"/gazebo/command/tilt_pos"};
  std::string fdmAddress{"127.0.0.1"};
  std::string listenAddress{"127.0.0.1"};
  uint16_t fdmPortIn{9002};
  uint16_t fdmPortOut{9003};
  int motorNum{0};
  int servoNum{0};
  int connectionTimeoutMaxCount{10};
  int connectionTimeoutCount{0};
  bool ardupilotOnline{false};
  bool socketsReady{false};
  bool entitiesResolved{false};
  bool warnedNoImu{false};

  gz::math::Pose3d modelXYZToAirplaneXForwardZDown;
  gz::math::Pose3d gazeboXYZToNED;

  std::array<float, kMaxMotors> motorSpeed{};
  std::array<float, kMaxMotors> servoSpeed{};
  UdpSocket socketIn;
  UdpSocket socketOut;
  std::chrono::steady_clock::time_point lastServoPacketWallTime{};
  bool haveServoPacketWallTime{false};

  gz::transport::Node gzNode;
  gz::transport::Node::Subscriber imuSubscriber;
  gz::msgs::IMU imuMessage;
  bool imuMessageValid{false};
  std::mutex imuMutex;

  rclcpp::Context::SharedPtr rosContext;
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motorPublisher;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr servoPublisher;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> rosExecutor;
  std::thread rosThread;

  void OnImu(const gz::msgs::IMU &_message)
  {
    std::lock_guard<std::mutex> lock(imuMutex);
    imuMessage = _message;
    imuMessageValid = true;
  }
};

/////////////////////////////////////////////////
ArduRotorTiltPlugin::ArduRotorTiltPlugin()
  : dataPtr(new Private)
{
  dataPtr->servoSpeed.fill(0.5f);
}

/////////////////////////////////////////////////
ArduRotorTiltPlugin::~ArduRotorTiltPlugin()
{
  ShutdownRos();
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  dataPtr->modelEntity = _entity;
  dataPtr->model = gz::sim::Model(_entity);
  if (!dataPtr->model.Valid(_ecm))
  {
    gzerr << "[ArduRotorTiltPlugin] plugin must be attached to a model\n";
    return;
  }

  dataPtr->modelName = dataPtr->model.Name(_ecm);
  dataPtr->fdmAddress = _sdf->Get(
      "fdm_addr", std::string("127.0.0.1")).first;
  dataPtr->listenAddress = _sdf->Get(
      "listen_addr", std::string("127.0.0.1")).first;
  dataPtr->fdmPortIn = _sdf->Get("fdm_port_in", uint32_t(9002)).first;
  dataPtr->fdmPortOut = _sdf->Get("fdm_port_out", uint32_t(9003)).first;
  dataPtr->motorNum = _sdf->Get("motor_num", 0).first;
  dataPtr->servoNum = _sdf->Get("servo_num", 0).first;
  dataPtr->motorTopic = NormalizeTopic(_sdf->Get(
      "motor_pub", std::string("/gazebo/command/prop_speed")).first);
  dataPtr->servoTopic = NormalizeTopic(_sdf->Get(
      "servo_pub", std::string("/gazebo/command/tilt_pos")).first);
  dataPtr->imuName = _sdf->Get(
      "imuName", std::string("imu_sensor")).first;
  dataPtr->connectionTimeoutMaxCount = _sdf->Get(
      "connectionTimeoutMaxCount", 10).first;

  dataPtr->motorNum = std::clamp(dataPtr->motorNum, 0,
                                 static_cast<int>(kMaxMotors));
  dataPtr->servoNum = std::clamp(dataPtr->servoNum, 0,
                                 static_cast<int>(kMaxMotors));

  dataPtr->modelXYZToAirplaneXForwardZDown = gz::math::Pose3d::Zero;
  if (_sdf->HasElement("modelXYZToAirplaneXForwardZDown"))
  {
    dataPtr->modelXYZToAirplaneXForwardZDown = _sdf->Get<gz::math::Pose3d>(
        "modelXYZToAirplaneXForwardZDown");
  }
  dataPtr->gazeboXYZToNED = gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (_sdf->HasElement("gazeboXYZToNED"))
  {
    dataPtr->gazeboXYZToNED = _sdf->Get<gz::math::Pose3d>("gazeboXYZToNED");
  }

  if (!dataPtr->socketIn.Bind(dataPtr->listenAddress, dataPtr->fdmPortIn))
  {
    gzerr << "[" << dataPtr->modelName << "] failed to bind "
          << dataPtr->listenAddress << ":" << dataPtr->fdmPortIn << "\n";
    return;
  }
  if (!dataPtr->socketOut.Connect(dataPtr->fdmAddress, dataPtr->fdmPortOut))
  {
    gzerr << "[" << dataPtr->modelName << "] failed to connect "
          << dataPtr->fdmAddress << ":" << dataPtr->fdmPortOut << "\n";
    return;
  }
  dataPtr->socketsReady = true;

  InitializeRos();
  gzmsg << "[" << dataPtr->modelName
        << "] ArduRotorTiltPlugin configured; waiting for IMU entity\n";
}

/////////////////////////////////////////////////
bool ArduRotorTiltPlugin::ResolveEntities(
    gz::sim::EntityComponentManager &_ecm)
{
  if (dataPtr->entitiesResolved)
    return true;

  const auto findTyped = [&_ecm](const std::string &_name,
                                 gz::sim::ComponentTypeId _type,
                                 gz::sim::Entity _relative) {
    std::vector<gz::sim::Entity> result;
    auto matches = gz::sim::entitiesFromScopedName(_name, _ecm, _relative);
    if (matches.empty())
      matches = gz::sim::entitiesFromScopedName(_name, _ecm);
    for (const auto entity : matches)
    {
      if (_ecm.EntityHasComponentType(entity, _type))
        result.push_back(entity);
    }
    return result;
  };

  auto imuMatches = findTyped(dataPtr->imuName,
                              gz::sim::components::Imu::typeId,
                              dataPtr->modelEntity);
  if (imuMatches.empty())
    imuMatches = findTyped(LastScopePart(dataPtr->imuName),
                           gz::sim::components::Imu::typeId,
                           dataPtr->modelEntity);
  if (imuMatches.empty())
  {
    if (!dataPtr->warnedNoImu)
    {
      gzdbg << "[" << dataPtr->modelName << "] waiting for IMU ["
            << dataPtr->imuName << "]\n";
      dataPtr->warnedNoImu = true;
    }
    return false;
  }

  dataPtr->imuSensor = imuMatches.front();
  dataPtr->imuLink = _ecm.ParentEntity(dataPtr->imuSensor);
  if (!_ecm.EntityHasComponentType(
      dataPtr->imuLink, gz::sim::components::Link::typeId))
  {
    gzerr << "[" << dataPtr->modelName << "] IMU parent is not a link\n";
    return false;
  }

  dataPtr->baseLink = dataPtr->model.CanonicalLink(_ecm);
  if (dataPtr->baseLink == gz::sim::kNullEntity)
    dataPtr->baseLink = dataPtr->imuLink;
  if (dataPtr->baseLink == gz::sim::kNullEntity)
    return false;

  gz::sim::Link(dataPtr->baseLink).EnableVelocityChecks(_ecm);
  gz::sim::Link(dataPtr->imuLink).EnableVelocityChecks(_ecm);

  std::string imuTopic;
  const auto *imuComponent = _ecm.Component<gz::sim::components::Imu>(
      dataPtr->imuSensor);
  if (imuComponent != nullptr)
    imuTopic = imuComponent->Data().Topic();
  if (imuTopic.empty())
  {
    imuTopic = gz::sim::scopedName(dataPtr->imuSensor, _ecm);
    if (imuTopic.empty() || imuTopic.front() != '/')
      imuTopic.insert(imuTopic.begin(), '/');
    imuTopic += "/imu";
  }
  if (!dataPtr->gzNode.Subscribe(imuTopic, &Private::OnImu, dataPtr.get()))
  {
    gzerr << "[" << dataPtr->modelName << "] failed to subscribe to IMU "
          << imuTopic << "\n";
    return false;
  }
  dataPtr->entitiesResolved = true;
  gzmsg << "[" << dataPtr->modelName << "] resolved IMU topic " << imuTopic
        << " and canonical body link\n";
  return true;
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !dataPtr->socketsReady)
    return;
  if (!ResolveEntities(_ecm))
    return;

  ReceiveMotorCommand();
  PublishActuators();
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused || !dataPtr->entitiesResolved ||
      !dataPtr->ardupilotOnline)
    return;
  SendState(_info, _ecm);
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::ReceiveMotorCommand()
{
  if (!dataPtr->socketsReady)
    return;

  const auto now = std::chrono::steady_clock::now();
  ServoPacket packet{};
  bool received = false;
  while (true)
  {
    const ssize_t size = dataPtr->socketIn.Receive(&packet, sizeof(packet));
    if (size < 0)
      break;
    const std::size_t expectedChannels = static_cast<std::size_t>(
        std::max(0, dataPtr->motorNum + dataPtr->servoNum));
    if (size < static_cast<ssize_t>(sizeof(float) * expectedChannels))
    {
      gzerr << "[" << dataPtr->modelName
            << "] short ArduPilot servo packet: " << size << " bytes\n";
      continue;
    }
    received = true;
    for (std::size_t i = 0; i < kMaxMotors; ++i)
    {
      if (i < static_cast<std::size_t>(dataPtr->motorNum))
      {
        const double command = std::clamp(
            static_cast<double>(packet.motorSpeed[i]), -1.0, 1.0);
        dataPtr->motorSpeed[i] = static_cast<float>(command * 1000.0);
      }
      else if (i < static_cast<std::size_t>(dataPtr->motorNum +
                                             dataPtr->servoNum))
      {
        const double command = std::clamp(
            static_cast<double>(packet.motorSpeed[i]), -2.0, 2.0);
        const std::size_t servo = i - static_cast<std::size_t>(
            dataPtr->motorNum);
        dataPtr->servoSpeed[servo] = static_cast<float>(
            (command - 0.5) * 1000.0 + 1500.0);
      }
    }
  }

  if (received)
  {
    dataPtr->connectionTimeoutCount = 0;
    dataPtr->ardupilotOnline = true;
    dataPtr->lastServoPacketWallTime = now;
    dataPtr->haveServoPacketWallTime = true;
    return;
  }

  if (!dataPtr->ardupilotOnline || !dataPtr->haveServoPacketWallTime)
    return;
  const auto elapsed = now - dataPtr->lastServoPacketWallTime;
  dataPtr->connectionTimeoutCount = static_cast<int>(
      std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
  if (dataPtr->connectionTimeoutCount >
      std::max(1, dataPtr->connectionTimeoutMaxCount))
  {
    dataPtr->connectionTimeoutCount = 0;
    dataPtr->ardupilotOnline = false;
    dataPtr->haveServoPacketWallTime = false;
    dataPtr->motorSpeed.fill(0.0f);
    dataPtr->servoSpeed.fill(1500.0f);
    gzwarn << "[" << dataPtr->modelName
           << "] ArduPilot connection timed out; resetting actuator command\n";
  }
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::PublishActuators()
{
  if (!dataPtr->ardupilotOnline || !dataPtr->motorPublisher)
    return;

  mav_msgs::msg::Actuators motorMessage;
  if (dataPtr->motorNum == 4)
  {
    // Exact ROS 1 Tilt ordering: [1, 4, 2, 3].
    for (const int index : {0, 3, 1, 2})
      motorMessage.angular_velocities.push_back(dataPtr->motorSpeed[index]);
  }
  else
  {
    for (int i = 0; i < dataPtr->motorNum; ++i)
      motorMessage.angular_velocities.push_back(dataPtr->motorSpeed[i]);
  }
  dataPtr->motorPublisher->publish(motorMessage);

  if (!dataPtr->servoPublisher)
    return;

  mav_msgs::msg::Actuators servoMessage;
  // The ROS 1 source converts the PWM-like value in place before publishing;
  // retain that observable behavior and its four-channel order.
  for (int i = 0; i < dataPtr->servoNum; ++i)
  {
    dataPtr->servoSpeed[i] = static_cast<float>(
        (dataPtr->servoSpeed[i] - 1500.0) / 1000.0 * 135.0 / 57.3);
  }
  if (dataPtr->servoNum == 4)
  {
    for (const int index : {0, 3, 1, 2})
      servoMessage.angular_velocities.push_back(dataPtr->servoSpeed[index]);
  }
  else
  {
    for (int i = 0; i < dataPtr->servoNum; ++i)
      servoMessage.angular_velocities.push_back(dataPtr->servoSpeed[i]);
  }
  dataPtr->servoPublisher->publish(servoMessage);
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::SendState(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  gz::msgs::IMU imu;
  {
    std::lock_guard<std::mutex> lock(dataPtr->imuMutex);
    if (!dataPtr->imuMessageValid)
      return;
    imu = dataPtr->imuMessage;
  }

  const auto pose = gz::sim::Link(dataPtr->baseLink).WorldPose(_ecm);
  const auto velocity = gz::sim::Link(dataPtr->baseLink).WorldLinearVelocity(
      _ecm);
  if (!pose || !velocity)
    return;

  const gz::math::Pose3d modelPose =
      dataPtr->modelXYZToAirplaneXForwardZDown + *pose;
  const gz::math::Pose3d nedPose = modelPose - dataPtr->gazeboXYZToNED;
  const gz::math::Vector3d velocityNed =
      dataPtr->gazeboXYZToNED.Rot().RotateVectorReverse(*velocity);

  FdmPacket packet{};
  packet.timestamp = std::chrono::duration<double>(_info.simTime).count();
  packet.imuAngularVelocityRPY[0] = imu.angular_velocity().x();
  packet.imuAngularVelocityRPY[1] = imu.angular_velocity().y();
  packet.imuAngularVelocityRPY[2] = imu.angular_velocity().z();
  packet.imuLinearAccelerationXYZ[0] = imu.linear_acceleration().x();
  packet.imuLinearAccelerationXYZ[1] = imu.linear_acceleration().y();
  packet.imuLinearAccelerationXYZ[2] = imu.linear_acceleration().z();
  packet.imuOrientationQuat[0] = nedPose.Rot().W();
  packet.imuOrientationQuat[1] = nedPose.Rot().X();
  packet.imuOrientationQuat[2] = nedPose.Rot().Y();
  packet.imuOrientationQuat[3] = nedPose.Rot().Z();
  packet.velocityXYZ[0] = velocityNed.X();
  packet.velocityXYZ[1] = velocityNed.Y();
  packet.velocityXYZ[2] = velocityNed.Z();
  packet.positionXYZ[0] = nedPose.Pos().X();
  packet.positionXYZ[1] = nedPose.Pos().Y();
  packet.positionXYZ[2] = nedPose.Pos().Z();
  (void)dataPtr->socketOut.Send(&packet, sizeof(packet));
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::InitializeRos()
{
  try
  {
    dataPtr->rosContext = std::make_shared<rclcpp::Context>();
    dataPtr->rosContext->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(dataPtr->rosContext);
    dataPtr->rosNode = std::make_shared<rclcpp::Node>(
        dataPtr->modelName + "_plugin", options);
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    dataPtr->motorPublisher = dataPtr->rosNode->create_publisher<
        mav_msgs::msg::Actuators>(dataPtr->motorTopic, qos);
    dataPtr->servoPublisher = dataPtr->rosNode->create_publisher<
        mav_msgs::msg::Actuators>(dataPtr->servoTopic, qos);

    rclcpp::ExecutorOptions executorOptions;
    executorOptions.context = dataPtr->rosContext;
    dataPtr->rosExecutor = std::make_unique<
        rclcpp::executors::SingleThreadedExecutor>(executorOptions);
    dataPtr->rosExecutor->add_node(dataPtr->rosNode);
    dataPtr->rosThread = std::thread([this]()
    {
      if (dataPtr->rosExecutor)
        dataPtr->rosExecutor->spin();
    });
  }
  catch (const std::exception &error)
  {
    gzerr << "[ArduRotorTiltPlugin] ROS 2 initialization failed: "
          << error.what() << "\n";
    ShutdownRos();
  }
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::ShutdownRos()
{
  if (dataPtr->rosExecutor)
    dataPtr->rosExecutor->cancel();
  if (dataPtr->rosContext && dataPtr->rosContext->is_valid())
    dataPtr->rosContext->shutdown("ArduRotorTiltPlugin shutdown");
  if (dataPtr->rosThread.joinable())
    dataPtr->rosThread.join();
  if (dataPtr->rosExecutor && dataPtr->rosNode)
    dataPtr->rosExecutor->remove_node(dataPtr->rosNode);
  dataPtr->motorPublisher.reset();
  dataPtr->servoPublisher.reset();
  dataPtr->rosNode.reset();
  dataPtr->rosExecutor.reset();
  dataPtr->rosContext.reset();
}

/////////////////////////////////////////////////
void ArduRotorTiltPlugin::Reset(
    const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager & /*_ecm*/)
{
  dataPtr->connectionTimeoutCount = 0;
  dataPtr->ardupilotOnline = false;
  dataPtr->haveServoPacketWallTime = false;
  dataPtr->motorSpeed.fill(0.0f);
  dataPtr->servoSpeed.fill(0.5f);
  std::lock_guard<std::mutex> lock(dataPtr->imuMutex);
  dataPtr->imuMessageValid = false;
}

}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::ArduRotorTiltPlugin, gz::sim::System,
  gazebo::ArduRotorTiltPlugin::ISystemConfigure,
  gazebo::ArduRotorTiltPlugin::ISystemPreUpdate,
  gazebo::ArduRotorTiltPlugin::ISystemPostUpdate,
  gazebo::ArduRotorTiltPlugin::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gazebo::ArduRotorTiltPlugin, "ArduRotorTiltPlugin")
