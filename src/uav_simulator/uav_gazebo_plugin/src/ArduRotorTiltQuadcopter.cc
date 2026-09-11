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
/*
 * Gazebo Sim port of the ROS 1 ArduRotorTiltQuadcopter.
 *
 * The wire protocol and the SDF contract below are the ones used by
 * ArduPilot's SITL/SIM_Gazebo backend.  In particular, this is deliberately
 * not the JSON ArduPilotPlugin and it is not a new vehicle-specific aero
 * plugin. Both versions publish ROS mav_msgs Actuators through the original
 * Gazebo ROS interface, which owns the conversion to RotorS transport.
 */

#include "ArduRotorTiltQuadcopter.hh"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <limits>
#include <string>
#include <unordered_set>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gz/math/Pose3.hh>
#include <mav_msgs/msg/actuators.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/sim/components/GpuLidar.hh>
#include <mavlink/v2.0/common/mavlink.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Lidar.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace
{

constexpr std::size_t kMaxMotors = 16;

/// The two packets are ABI-compatible with ArduPilot libraries/SITL/SIM_Gazebo.h.
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

/// UDP wrapper with a bounded receive wait, matching the ROS 1 handshake.
/// The descriptor stays non-blocking so draining queued packets never waits.
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

  bool Bind(const std::string &address, uint16_t port)
  {
    if (!Open())
      return false;

    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(port);
    if (::inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1)
      return false;

    return ::bind(fd_, reinterpret_cast<const sockaddr *>(&endpoint),
                  sizeof(endpoint)) == 0;
  }

  bool Connect(const std::string &address, uint16_t port)
  {
    if (!Open())
      return false;

    sockaddr_in endpoint{};
    endpoint.sin_family = AF_INET;
    endpoint.sin_port = htons(port);
    if (::inet_pton(AF_INET, address.c_str(), &endpoint.sin_addr) != 1)
      return false;

    return ::connect(fd_, reinterpret_cast<const sockaddr *>(&endpoint),
                     sizeof(endpoint)) == 0;
  }

  ssize_t Receive(void *buffer, std::size_t size, int timeoutMs = 0)
  {
    if (fd_ < 0)
      return -1;
    if (timeoutMs > 0)
    {
      pollfd descriptor{fd_, POLLIN, 0};
      if (::poll(&descriptor, 1, timeoutMs) <= 0)
        return -1;
    }
    const ssize_t result = ::recv(fd_, buffer, size, 0);
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      return -1;
    return result;
  }

  ssize_t Send(const void *buffer, std::size_t size) const
  {
    if (fd_ < 0)
      return -1;
    return ::send(fd_, buffer, size, 0);
  }

  ssize_t ReceiveFrom(void *buffer, std::size_t size, sockaddr_in &peer)
  {
    socklen_t length = sizeof(peer);
    return ::recvfrom(fd_, buffer, size, 0,
        reinterpret_cast<sockaddr *>(&peer), &length);
  }

  ssize_t SendTo(const void *buffer, std::size_t size, const sockaddr_in &peer)
  {
    return ::sendto(fd_, buffer, size, 0,
        reinterpret_cast<const sockaddr *>(&peer), sizeof(peer));
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

std::string NormalizeTopic(std::string topic)
{
  if (topic.empty())
    return topic;
  if (topic.front() != '/')
    topic.insert(topic.begin(), '/');
  return topic;
}

std::string LastScopePart(const std::string &name)
{
  const std::size_t separator = name.rfind("::");
  if (separator == std::string::npos)
    return name;
  return name.substr(separator + 2);
}

}  // namespace

namespace gazebo
{

class ArduRotorTiltQuadcopter::Private
{
public:
  ~Private()
  {
    motorPublisher.reset();
    servoPublisher.reset();
    rosNode.reset();
    // Never shut down another plugin's ROS context when this model unloads.
    if (rosContext && rosContext->is_valid())
      rosContext->shutdown("ArduRotorTiltQuadcopter unloaded");
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  gz::sim::Entity baseLink{gz::sim::kNullEntity};
  gz::sim::Entity imuLink{gz::sim::kNullEntity};
  gz::sim::Entity imuSensor{gz::sim::kNullEntity};
  std::string modelName;
  std::string imuName{"imu_sensor"};
  std::string motorTopic{"/gazebo/command/prop_speed"};

  std::string fdmAddress{"127.0.0.1"};
  std::string listenAddress{"127.0.0.1"};
  uint16_t fdmPortIn{9002};
  uint16_t fdmPortOut{9003};

  int motorNum{0};
  int servoNum{0};
  int connectionTimeoutCount{0};
  int connectionTimeoutMaxCount{10};
  bool ardupilotOnline{false};
  bool entitiesResolved{false};
  bool socketsReady{false};
  bool warnedNoImu{false};
  double lastSimTime{0.0};

  // Gazebo Sim can execute many update iterations per wall-clock second when
  // the world is run with ``-r`` and no real-time cap.  The ROS 1 plugin's
  // timeout counter was advanced by a blocking one-second receive call.  A
  // non-blocking port must therefore measure the same interval in wall time,
  // rather than incrementing once per (possibly much faster) simulation
  // iteration.
  std::chrono::steady_clock::time_point lastServoPacketWallTime{};
  bool haveServoPacketWallTime{false};

  gz::math::Pose3d modelXYZToAirplaneXForwardZDown;
  gz::math::Pose3d gazeboXYZToNED;

  std::array<float, kMaxMotors> motorSpeed{};
  UdpSocket socketIn;
  UdpSocket socketOut;

  gz::transport::Node node;
  std::shared_ptr<rclcpp::Context> rosContext;
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motorPublisher;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr servoPublisher;
  std::string servoTopic;
  std::string rangeName;
  std::string rangeBindAddress;
  uint32_t rangeBindPort{9025};
  double rangeMin{0.05}, rangeMax{10.0}, rangeRate{50.0}, rangeLastTime{0.0};
  UdpSocket rangeSocket;
  sockaddr_in rangePeer{};
  bool rangeEnabled{false}, rangePeerKnown{false};
  double rangeValue{0.0};
  uint64_t rangeSampleCount{0};
  std::mutex rangeMutex;
  mavlink_status_t rangeMavlinkStatus{};

  void OnRange(const gz::msgs::LaserScan &message)
  {
    std::lock_guard<std::mutex> lock(rangeMutex);
    rangeValue = message.ranges_size() ? message.ranges(0) :
        std::numeric_limits<double>::quiet_NaN();
    if ((++rangeSampleCount % 100) == 1)
      gzmsg << "[ArduRotorTiltQuadcopter] range sample " << rangeValue << " m\n";
  }

  void SendForwardRangefinder(double simTime)
  {
    if (!rangeEnabled)
      return;
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> request{};
    sockaddr_in peer{};
    while (rangeSocket.ReceiveFrom(request.data(), request.size(), peer) >= 0)
    {
      rangePeer = peer;
      rangePeerKnown = true;
    }
    if (!rangePeerKnown || simTime - rangeLastTime < 1.0 / std::max(rangeRate, 1.0))
      return;
    rangeLastTime = simTime;
    double distance;
    {
      std::lock_guard<std::mutex> lock(rangeMutex);
      distance = rangeValue;
    }
    const bool valid = std::isfinite(distance) && distance >= rangeMin && distance <= rangeMax;
    mavlink_distance_sensor_t sensor{};
    sensor.time_boot_ms = static_cast<uint32_t>(simTime * 1000.0);
    sensor.min_distance = static_cast<uint16_t>(std::llround(rangeMin * 100.0));
    sensor.max_distance = static_cast<uint16_t>(std::llround(rangeMax * 100.0));
    sensor.current_distance = valid ? static_cast<uint16_t>(std::llround(distance * 100.0)) :
        sensor.max_distance + 1U;
    sensor.type = MAV_DISTANCE_SENSOR_LASER;
    sensor.orientation = MAV_SENSOR_ROTATION_NONE;
    sensor.covariance = UINT8_MAX;
    sensor.signal_quality = valid ? 100U : 0U;
    mavlink_message_t message;
    mavlink_msg_distance_sensor_encode_status(32, MAV_COMP_ID_PERIPHERAL,
        &rangeMavlinkStatus, &message, &sensor);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t size = mavlink_msg_to_send_buffer(buffer, &message);
    rangeSocket.SendTo(buffer, size, rangePeer);
  }
  gz::transport::Node::Subscriber imuSubscriber;
  gz::msgs::IMU imuMessage;
  bool imuMessageValid{false};
  std::mutex imuMutex;

  void OnImu(const gz::msgs::IMU &_message)
  {
    std::lock_guard<std::mutex> lock(imuMutex);
    imuMessage = _message;
    imuMessageValid = true;
  }
};

/////////////////////////////////////////////////
ArduRotorTiltQuadcopter::ArduRotorTiltQuadcopter()
  : dataPtr(new Private)
{
}

/////////////////////////////////////////////////
ArduRotorTiltQuadcopter::~ArduRotorTiltQuadcopter() = default;

/////////////////////////////////////////////////
void ArduRotorTiltQuadcopter::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  dataPtr->model = gz::sim::Model(_entity);
  if (!dataPtr->model.Valid(_ecm))
  {
    gzerr << "[ArduRotorTiltQuadcopter] plugin must be attached to a model\n";
    return;
  }

  dataPtr->modelName = dataPtr->model.Name(_ecm);

  // Keep the exact ROS 1 SDF defaults and key names.
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
  dataPtr->imuName = _sdf->Get(
      "imuName", std::string("imu_sensor")).first;
  dataPtr->connectionTimeoutMaxCount = _sdf->Get(
      "connectionTimeoutMaxCount", 10).first;

  dataPtr->motorNum = std::clamp(dataPtr->motorNum, 0,
                                 static_cast<int>(kMaxMotors));
  dataPtr->servoNum = std::clamp(dataPtr->servoNum, 0,
                                 static_cast<int>(kMaxMotors));

  if (dataPtr->motorNum != 8 || dataPtr->servoNum != 4)
  {
    gzerr << "[ArduRotorTiltQuadcopter] requires the original 8 motor + 4 servo channels\n";
    return;
  }
  dataPtr->servoTopic = NormalizeTopic(_sdf->Get(
      "servo1_pub", std::string("/gazebo/command/tilt1_pos")).first);
  dataPtr->rangeName = _sdf->Get("rangefinderName", std::string("front_rangefinder")).first;
  dataPtr->rangeBindAddress = _sdf->Get("rangefinderUdpBindAddr", std::string("127.0.0.1")).first;
  dataPtr->rangeBindPort = _sdf->Get("rangefinderUdpBindPort", uint32_t(9025)).first;
  dataPtr->rangeMin = _sdf->Get("rangefinderMinDistanceM", 0.05).first;
  dataPtr->rangeMax = _sdf->Get("rangefinderMaxDistanceM", 10.0).first;
  dataPtr->rangeRate = _sdf->Get("rangefinderRateHz", 50.0).first;

  // ROS 1 defaults this transform to identity.  The tilt_quadcopter SDF supplies
  // its explicit pi-roll value, so keep the default unchanged for every
  // other original model that omits the optional element.
  dataPtr->modelXYZToAirplaneXForwardZDown = gz::math::Pose3d(
      0, 0, 0, 0, 0, 0);
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
          << dataPtr->listenAddress << ":" << dataPtr->fdmPortIn
          << " for ArduPilot servo packets\n";
    return;
  }
  if (!dataPtr->socketOut.Connect(dataPtr->fdmAddress,
                                 dataPtr->fdmPortOut))
  {
    gzerr << "[" << dataPtr->modelName << "] failed to connect "
          << dataPtr->fdmAddress << ":" << dataPtr->fdmPortOut
          << " for ArduPilot FDM packets\n";
    return;
  }
  try
  {
    // Restore ROS 1's public ROS command topics. The original world bridge
    // owns ROS-to-Gazebo conversion; bypassing it changes the command path.
    dataPtr->rosContext = std::make_shared<rclcpp::Context>();
    dataPtr->rosContext->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(dataPtr->rosContext);
    dataPtr->rosNode = std::make_shared<rclcpp::Node>(
        dataPtr->modelName + "_plugin", options);
    dataPtr->motorPublisher = dataPtr->rosNode->create_publisher<
        mav_msgs::msg::Actuators>(dataPtr->motorTopic, rclcpp::QoS(10));
    dataPtr->servoPublisher = dataPtr->rosNode->create_publisher<
        mav_msgs::msg::Actuators>(dataPtr->servoTopic, rclcpp::QoS(10));
  }
  catch (const std::exception &error)
  {
    gzerr << "[" << dataPtr->modelName << "] ROS command publisher setup failed: "
          << error.what() << "\n";
    return;
  }
  dataPtr->socketsReady = true;

  // Nested model and sensor entities are not guaranteed to exist during
  // Configure().  ResolveEntities() is therefore called from PreUpdate().
  gzmsg << "[" << dataPtr->modelName << "] ArduRotorTiltQuadcopter configured; "
        << "waiting for nested IMU entity\n";
}

/////////////////////////////////////////////////
bool ArduRotorTiltQuadcopter::ResolveEntities(
    gz::sim::EntityComponentManager &_ecm)
{
  if (dataPtr->entitiesResolved)
    return true;

  const auto findTyped = [&_ecm](const std::string &name,
                                 gz::sim::ComponentTypeId _type,
                                 gz::sim::Entity _relative) {
    std::vector<gz::sim::Entity> result;
    // Search only descendants of this vehicle, including nested models.
    // A global first-IMU fallback can silently bind a different aircraft.
    _ecm.Each<gz::sim::components::Name>(
        [&](gz::sim::Entity entity, const gz::sim::components::Name *component) {
      if (!_ecm.EntityHasComponentType(entity, _type))
        return true;
      auto parent = entity;
      while (parent != gz::sim::kNullEntity && parent != _relative)
        parent = _ecm.ParentEntity(parent);
      if (parent != _relative)
        return true;
      const auto scoped = gz::sim::scopedName(entity, _ecm, "::", false);
      if (component->Data() == name || scoped == name ||
          (scoped.size() > name.size() + 2 &&
           scoped.compare(scoped.size() - name.size() - 2,
                          name.size() + 2, "::" + name) == 0))
        result.push_back(entity);
      return true;
    });
    return result;
  };

  // The name is intentionally the same scoped name used by the ROS 1 SDF.
  auto imuMatches = findTyped(dataPtr->imuName,
                              gz::sim::components::Imu::typeId,
                              dataPtr->model.Entity());
  if (imuMatches.empty())
  {
    imuMatches = findTyped(LastScopePart(dataPtr->imuName),
                           gz::sim::components::Imu::typeId,
                           dataPtr->model.Entity());
  }
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
  const gz::sim::Entity imuParent = _ecm.ParentEntity(dataPtr->imuSensor);
  if (!_ecm.EntityHasComponentType(
      imuParent, gz::sim::components::Link::typeId))
  {
    gzerr << "[" << dataPtr->modelName << "] parent of IMU ["
          << dataPtr->imuName << "] is not a link\n";
    return false;
  }
  dataPtr->imuLink = imuParent;

  // Find the original canonical body link by its unscoped name.  This keeps
  // the nested tilt_quadcopter_base hierarchy intact while avoiding a new SDF key.
  auto baseMatches = findTyped("base",
                               gz::sim::components::Link::typeId,
                               dataPtr->model.Entity());
  if (baseMatches.empty())
  {
    gzerr << "[" << dataPtr->modelName
          << "] cannot find nested link [base]\n";
    return false;
  }
  dataPtr->baseLink = baseMatches.front();

  gz::sim::Link(dataPtr->baseLink).EnableVelocityChecks(_ecm);
  gz::sim::Link(dataPtr->imuLink).EnableVelocityChecks(_ecm);

  // Keep the topic from the original SDF when one is specified.  The ROS 1
  // model intentionally uses the literal ``__default_topic__``; Gazebo Sim
  // publishes that value verbatim, so deriving a scoped topic here would
  // leave the plugin subscribed to a topic with no publisher.  For models
  // without an explicit topic, use Gazebo Sim's scoped sensor topic.
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
  if (!dataPtr->node.Subscribe(imuTopic, &Private::OnImu, dataPtr.get()))
  {
    gzerr << "[" << dataPtr->modelName << "] failed to subscribe to IMU "
          << imuTopic << "\n";
    return false;
  }

  // The baseline tilt_quadcopter has no rangefinder. User-approved (2026-09-06):
  // disable only range sending when absent, never invent a sensor in the model.
  auto rangeMatches = findTyped(dataPtr->rangeName,
      gz::sim::components::GpuLidar::typeId, dataPtr->model.Entity());
  if (rangeMatches.empty())
    rangeMatches = findTyped(dataPtr->rangeName,
        gz::sim::components::Lidar::typeId, dataPtr->model.Entity());
  if (!rangeMatches.empty())
  {
    std::string topic;
    if (const auto *gpu = _ecm.Component<gz::sim::components::GpuLidar>(rangeMatches.front()))
      topic = gpu->Data().Topic();
    else if (const auto *lidar = _ecm.Component<gz::sim::components::Lidar>(rangeMatches.front()))
      topic = lidar->Data().Topic();
    if (topic.empty())
      topic = "/" + gz::sim::scopedName(rangeMatches.front(), _ecm, "/", true) + "/scan";
    gzmsg << "[" << dataPtr->modelName << "] resolved range topic " << topic << "\n";
    if (!dataPtr->rangeSocket.Bind(dataPtr->rangeBindAddress, dataPtr->rangeBindPort))
    {
      gzerr << "[ArduRotorTiltQuadcopter] could not bind rangefinder socket\n";
      return false;
    }
    // ArduPilot SERIAL7 listens on the next port and may not emit a packet
    // for peer discovery. Seed the destination so range transmission starts
    // immediately after the first lidar sample.
    dataPtr->rangePeer.sin_family = AF_INET;
    dataPtr->rangePeer.sin_port = htons(9026);
    dataPtr->rangePeer.sin_addr.s_addr = inet_addr(dataPtr->rangeBindAddress.c_str());
    dataPtr->rangePeerKnown = true;
    dataPtr->rangeEnabled = dataPtr->node.Subscribe(topic, &Private::OnRange, dataPtr.get());
  }
  else
  {
    gzwarn << "[ArduRotorTiltQuadcopter] no rangefinder in model; range sending disabled\n";
  }

  dataPtr->entitiesResolved = true;
  gzmsg << "[" << dataPtr->modelName << "] resolved IMU topic "
        << imuTopic << " and body link base\n";
  return true;
}

/////////////////////////////////////////////////
void ArduRotorTiltQuadcopter::ReceiveMotorCommand()
{
  if (!dataPtr->socketsReady)
    return;

  const auto now = std::chrono::steady_clock::now();
  ServoPacket packet{};
  bool received = false;
  // Match ROS 1 ReceiveMotorCommand: wait for the next controller packet
  // before advancing physics (1 ms offline, 1 s once SITL is online), then
  // drain queued packets without waiting. A wholly non-blocking receive
  // allows Gazebo to advance repeatedly on stale actuator commands.
  int waitMs = dataPtr->ardupilotOnline ? 1000 : 1;
  while (true)
  {
    const ssize_t size = dataPtr->socketIn.Receive(&packet, sizeof(packet), waitMs);
    waitMs = 0;
    if (size < 0)
      break;
    const std::size_t expectedChannels = static_cast<std::size_t>(
        std::max(0, dataPtr->motorNum + dataPtr->servoNum));
    if (size < static_cast<ssize_t>(sizeof(float) * expectedChannels))
    {
      gzerr << "[" << dataPtr->modelName << "] short ArduPilot servo packet: "
            << size << " bytes\n";
      continue;
    }
    received = true;
    // Keep the latest packet if several SITL updates accumulated.
    for (std::size_t i = 0; i < kMaxMotors; ++i)
      dataPtr->motorSpeed[i] = packet.motorSpeed[i];
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

  // In ROS 1, once ArduPilot was detected, Recv() waited up to one second
  // before one missed receive was counted.  Preserve that observable timeout
  // contract with the bounded receive wait above.
  const auto elapsed = now - dataPtr->lastServoPacketWallTime;
  dataPtr->connectionTimeoutCount = static_cast<int>(
      std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
  const int timeoutMaxCount = std::max(1, dataPtr->connectionTimeoutMaxCount);
  if (dataPtr->connectionTimeoutCount > timeoutMaxCount)
  {
    dataPtr->connectionTimeoutCount = 0;
    dataPtr->ardupilotOnline = false;
    dataPtr->haveServoPacketWallTime = false;
    dataPtr->motorSpeed.fill(0.0f);
    gzwarn << "[" << dataPtr->modelName
           << "] ArduPilot connection timed out; resetting motor command\n";
  }
}

/////////////////////////////////////////////////
void ArduRotorTiltQuadcopter::PublishMotorCommand()
{
  if (!dataPtr->motorPublisher || !dataPtr->ardupilotOnline)
    return;

  mav_msgs::msg::Actuators message;
  // ROS 1 ApplyMotorForces emits all eight motors in order; no quad remap.
  for (int i = 0; i < dataPtr->motorNum; ++i)
  {
    // Keep the original float array's rounding before publishing doubles.
    const double command = std::clamp(dataPtr->motorSpeed[i], 0.0f, 1.0f);
    const float speed = command * 1000.0f;
    message.angular_velocities.push_back(
        speed);
  }
  dataPtr->motorPublisher->publish(message);

  // Preserve the legacy PWM conversion including its 57.3 degree/radian factor.
  // Actuators.angular_velocities carries the original position references.
  mav_msgs::msg::Actuators servo;
  for (int i = 0; i < dataPtr->servoNum; ++i)
  {
    const double command = std::clamp(dataPtr->motorSpeed[dataPtr->motorNum + i], -2.0f, 2.0f);
    const float pwm = (command - 0.5f) * 1000.0f + 1500.0f;
    const float position = (pwm - 1500.0f) / 1000.0f * 135.0f / 57.3f;
    servo.angular_velocities.push_back(position);
  }
  dataPtr->servoPublisher->publish(servo);
}

/////////////////////////////////////////////////
void ArduRotorTiltQuadcopter::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (!dataPtr->entitiesResolved && !ResolveEntities(_ecm))
    return;

  dataPtr->SendForwardRangefinder(std::chrono::duration<double>(_info.simTime).count());
  ReceiveMotorCommand();
  PublishMotorCommand();
  // ROS 1 sends FDM in WorldUpdateBegin, before integrating the next step.
  // PostUpdate would combine newly integrated pose with an asynchronous IMU
  // sample that may still describe the previous step.
  if (dataPtr->ardupilotOnline && dataPtr->socketsReady)
    SendState(_info, _ecm);
  dataPtr->lastSimTime = std::chrono::duration<double>(_info.simTime).count();
}

/////////////////////////////////////////////////
void ArduRotorTiltQuadcopter::SendState(
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

  const auto pose = gz::sim::worldPose(dataPtr->baseLink, _ecm);
  auto velocity = gz::sim::Link(dataPtr->baseLink).WorldLinearVelocity(_ecm);
  if (!velocity)
    return;

  // Keep the ROS 1 Pose3d composition and frame convention unchanged.
  const gz::math::Pose3d gazeboXYZToModelXForwardZDown =
      dataPtr->modelXYZToAirplaneXForwardZDown + pose;
  const gz::math::Pose3d nedToModelXForwardZUp =
      gazeboXYZToModelXForwardZDown - dataPtr->gazeboXYZToNED;

  FdmPacket packet{};
  packet.timestamp = std::chrono::duration<double>(_info.simTime).count();
  packet.imuAngularVelocityRPY[0] = imu.angular_velocity().x();
  packet.imuAngularVelocityRPY[1] = imu.angular_velocity().y();
  packet.imuAngularVelocityRPY[2] = imu.angular_velocity().z();
  packet.imuLinearAccelerationXYZ[0] = imu.linear_acceleration().x();
  packet.imuLinearAccelerationXYZ[1] = imu.linear_acceleration().y();
  packet.imuLinearAccelerationXYZ[2] = imu.linear_acceleration().z();

  packet.imuOrientationQuat[0] = nedToModelXForwardZUp.Rot().W();
  packet.imuOrientationQuat[1] = nedToModelXForwardZUp.Rot().X();
  packet.imuOrientationQuat[2] = nedToModelXForwardZUp.Rot().Y();
  packet.imuOrientationQuat[3] = nedToModelXForwardZUp.Rot().Z();

  const gz::math::Vector3d velocityNED =
      dataPtr->gazeboXYZToNED.Rot().RotateVectorReverse(*velocity);
  packet.velocityXYZ[0] = velocityNED.X();
  packet.velocityXYZ[1] = velocityNED.Y();
  packet.velocityXYZ[2] = velocityNED.Z();

  packet.positionXYZ[0] = nedToModelXForwardZUp.Pos().X();
  packet.positionXYZ[1] = nedToModelXForwardZUp.Pos().Y();
  packet.positionXYZ[2] = nedToModelXForwardZUp.Pos().Z();

  (void)dataPtr->socketOut.Send(&packet, sizeof(packet));
}

/////////////////////////////////////////////////
void ArduRotorTiltQuadcopter::Reset(
    const gz::sim::UpdateInfo &/*_info*/,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
  dataPtr->connectionTimeoutCount = 0;
  dataPtr->ardupilotOnline = false;
  dataPtr->haveServoPacketWallTime = false;
  dataPtr->lastSimTime = 0.0;
  dataPtr->rangePeerKnown = false;
  dataPtr->rangeLastTime = 0.0;
  dataPtr->motorSpeed.fill(0.0f);
  std::lock_guard<std::mutex> lock(dataPtr->imuMutex);
  dataPtr->imuMessageValid = false;
}

}  // namespace gazebo

// Keep the SDF name exactly as in the ROS 1 model.
GZ_ADD_PLUGIN(gazebo::ArduRotorTiltQuadcopter, gz::sim::System,
  gazebo::ArduRotorTiltQuadcopter::ISystemConfigure,
  gazebo::ArduRotorTiltQuadcopter::ISystemPreUpdate,
  gazebo::ArduRotorTiltQuadcopter::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gazebo::ArduRotorTiltQuadcopter, "ArduRotorTiltQuadcopter")
GZ_ADD_PLUGIN_ALIAS(gazebo::ArduRotorTiltQuadcopter, "ArduRotorTiltQuadcopterFix")
