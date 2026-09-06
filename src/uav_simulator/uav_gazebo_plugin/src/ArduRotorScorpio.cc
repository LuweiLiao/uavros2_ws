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
// Gazebo Sim API port of the original Scorpio UDP + SocketCAN bridge.
// Keep the original packet ABI, channel mapping, conversion and ROS command path.
#include "ArduRotorScorpio.hh"
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <gz/math/Pose3.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Link.hh>
#include <gz/transport/Node.hh>
#include <mav_msgs/msg/actuators.hpp>
#include <rclcpp/rclcpp.hpp>

#define MAX_MOTORS 16
namespace {
struct ServoPacket { float motorSpeed[MAX_MOTORS]{}; };
struct fdmPacket {
    double timestamp;
    double imuAngularVelocityRPY[3];
    double imuLinearAccelerationXYZ[3];
    double imuOrientationQuat[4];
    double velocityXYZ[3];
    double positionXYZ[3];
};
static_assert(sizeof(ServoPacket) == 64, "Gazebo PWM packet ABI changed");
static_assert(sizeof(fdmPacket) == 136, "Gazebo FDM packet ABI changed");
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

  ssize_t Recv(void *buffer, std::size_t size, int timeoutMs = 0)
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


std::string CanonicalTopic(const std::string &name)
{
    std::string result = "/";
    for (const char c : name)
        if (c != '/' || result.back() != '/') result += c;
    return result;
}
}  // namespace

namespace gazebo {
class ArduRotorScorpio::Private {
public:
    ~Private()
    {
        if (!imu_topic.empty()) node.Unsubscribe(imu_topic);
        if (socketcan_fd >= 0) ::close(socketcan_fd);
        coxa_pub.reset(); femur_pub.reset(); tibia_pub.reset();
        motor_pub.reset(); servo_pub.reset(); rosNode.reset();
        if (rosContext && rosContext->is_valid())
            rosContext->shutdown("ArduRotorScorpio unloaded");
    }
    gz::sim::Model model{gz::sim::kNullEntity};
    gz::sim::Entity baseLink{gz::sim::kNullEntity};
    std::string modelName, imuName, imu_topic;
    bool ready{false}, entitiesResolved{false};
    double lastControllerUpdateTime{0.0};
    UdpSocket socket_in, socket_out;
    std::string fdm_addr, listen_addr;
    uint16_t fdm_port_in{9002}, fdm_port_out{9003};
    bool arduPilotOnline{false};
    int connectionTimeoutCount{0}, connectionTimeoutMaxCount{10};
    std::shared_ptr<rclcpp::Context> rosContext;
    rclcpp::Node::SharedPtr rosNode;
    rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr
        coxa_pub, femur_pub, tibia_pub, motor_pub, servo_pub;
    int motor_num{0}, servo_num{0}, coxa_num{0}, femur_num{0}, tibia_num{0};
    bool socketcan_enabled{false};
    std::string socketcan_interface{"vcan0"};
    int socketcan_fd{-1}, motor_channel_base{0}, servo_channel_base{0};
    uint8_t can_transfer_id{0}, can_source_node{0}, can_expected_toggle{0};
    uint8_t can_payload[64]{};
    size_t can_payload_len{0};
    bool can_transfer_active{false}, can_frame_logged{false}, can_command_logged{false};
    float motor_speed[MAX_MOTORS]{};
    float servo_speed[MAX_MOTORS], coxa_speed[MAX_MOTORS],
          femur_speed[MAX_MOTORS], tibia_speed[MAX_MOTORS];
    std::mutex imuMutex;
    gz::msgs::IMU imu;
    bool imuValid{false};
    gz::transport::Node node;
    void OnImu(const gz::msgs::IMU &message)
    {
        std::lock_guard<std::mutex> lock(imuMutex);
        imu = message;
        imuValid = true;
    }
};

ArduRotorScorpio::ArduRotorScorpio() : dataPtr(new Private)
{
    for (int i = 0; i < MAX_MOTORS; ++i) {
        dataPtr->coxa_speed[i] = dataPtr->femur_speed[i] =
            dataPtr->tibia_speed[i] = dataPtr->servo_speed[i] = 1500.0f;
    }
}
ArduRotorScorpio::~ArduRotorScorpio() = default;

void ArduRotorScorpio::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
{
    dataPtr->model = gz::sim::Model(_entity);
    if (!dataPtr->model.Valid(_ecm)) return;
    dataPtr->modelName = dataPtr->model.Name(_ecm);
    auto sdf = _sdf->Clone();
    dataPtr->motor_num = sdf->Get("motor_num", 0).first;
    dataPtr->servo_num = sdf->Get("servo_num", 0).first;
    dataPtr->coxa_num = sdf->Get("coxa_num", 0).first;
    dataPtr->femur_num = sdf->Get("femur_num", 0).first;
    dataPtr->tibia_num = sdf->Get("tibia_num", 0).first;
    for (int count : {dataPtr->motor_num, dataPtr->servo_num, dataPtr->coxa_num,
                      dataPtr->femur_num, dataPtr->tibia_num}) {
        if (count < 0 || count > MAX_MOTORS) {
            gzerr << "[ArduRotorScorpio] channel count outside original packet capacity\n";
            return;
        }
    }
    dataPtr->socketcan_enabled = sdf->Get("socketCanEnabled", false).first;
    dataPtr->socketcan_interface = sdf->Get("socketCanInterface", std::string("vcan0")).first;
    const int legs = dataPtr->coxa_num + dataPtr->femur_num + dataPtr->tibia_num;
    dataPtr->motor_channel_base = sdf->Get("motorChannelBase", legs).first;
    dataPtr->servo_channel_base = sdf->Get("servoChannelBase",
        dataPtr->motor_channel_base + dataPtr->motor_num).first;
    const int motorBase = dataPtr->socketcan_enabled ? dataPtr->motor_channel_base : legs;
    const int servoBase = dataPtr->socketcan_enabled ? dataPtr->servo_channel_base :
        legs + dataPtr->motor_num;
    if (motorBase < 0 || servoBase < 0 ||
        motorBase + dataPtr->motor_num > MAX_MOTORS ||
        servoBase + dataPtr->servo_num > MAX_MOTORS) {
        gzerr << "[ArduRotorScorpio] configured channels exceed the 16-channel UDP packet\n";
        return;
    }
    modelXYZToAirplaneXForwardZDown = sdf->Get("modelXYZToAirplaneXForwardZDown",
        gz::math::Pose3d::Zero).first;
    gazeboXYZToNED = sdf->Get("gazeboXYZToNED",
        gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0)).first;
    dataPtr->imuName = sdf->Get("imuName", std::string("imu_sensor")).first;
    dataPtr->connectionTimeoutMaxCount = sdf->Get("connectionTimeoutMaxCount", 10).first;
    if (dataPtr->socketcan_enabled && !InitSocketCAN()) {
        gzerr << "[" << dataPtr->modelName << "] failed to open SocketCAN interface ["
              << dataPtr->socketcan_interface << "]; leg commands will stay neutral.\n";
    }
    try {
        dataPtr->rosContext = std::make_shared<rclcpp::Context>();
        dataPtr->rosContext->init(0, nullptr);
        rclcpp::NodeOptions options;
        options.context(dataPtr->rosContext);
        dataPtr->rosNode = std::make_shared<rclcpp::Node>(
            dataPtr->modelName + "_plugin", options);
        auto publisher = [&](const char *key, const char *fallback) {
            return dataPtr->rosNode->create_publisher<mav_msgs::msg::Actuators>(
                CanonicalTopic(sdf->Get(key, std::string(fallback)).first), rclcpp::QoS(10));
        };
        dataPtr->coxa_pub = publisher("coxa_pub", "/gazebo/command/coxa_pos");
        dataPtr->femur_pub = publisher("femur_pub", "/gazebo/command/femur_pos");
        dataPtr->tibia_pub = publisher("tibia_pub", "/gazebo/command/tibia_pos");
        dataPtr->motor_pub = publisher("motor_pub", "/gazebo/command/prop_speed");
        dataPtr->servo_pub = publisher("servo_pub", "/gazebo/command/tilt_pos");
    } catch (const std::exception &e) {
        gzerr << "[ArduRotorScorpio] ROS publisher setup failed: " << e.what() << "\n";
        return;
    }
    if (!InitArduPilotSockets(sdf)) return;
    dataPtr->ready = true;
    gzmsg << "[" << dataPtr->modelName << "] ArduRotorScorpio configured: "
          << dataPtr->motor_num << " motors, " << dataPtr->servo_num << " tilt servos, "
          << dataPtr->coxa_num << "/" << dataPtr->femur_num << "/" << dataPtr->tibia_num
          << " leg joints; waiting for nested IMU\n";
}

bool ArduRotorScorpio::ResolveEntities(gz::sim::EntityComponentManager &_ecm)
{
    if (dataPtr->entitiesResolved) return true;
    const auto matches = gz::sim::entitiesFromScopedName(
        dataPtr->imuName, _ecm, dataPtr->model.Entity());
    for (auto entity : matches) {
        const auto *component = _ecm.Component<gz::sim::components::Imu>(entity);
        if (!component) continue;
        const auto imuLink = _ecm.ParentEntity(entity);
        // The original model's first body link belongs to the nested IMU model.
        dataPtr->baseLink = gz::sim::Model(_ecm.ParentEntity(imuLink)).LinkByName(_ecm, "base");
        if (dataPtr->baseLink == gz::sim::kNullEntity) return false;
        gz::sim::Link(dataPtr->baseLink).EnableVelocityChecks(_ecm);
        dataPtr->imu_topic = component->Data().Topic();
        if (dataPtr->imu_topic.empty())
            dataPtr->imu_topic = "/" + gz::sim::scopedName(entity, _ecm) + "/imu";
        if (!dataPtr->node.Subscribe(dataPtr->imu_topic, &Private::OnImu, dataPtr.get()))
            return false;
        dataPtr->entitiesResolved = true;
        gzmsg << "[" << dataPtr->modelName << "] resolved IMU "
              << dataPtr->imu_topic << "; ArduPilot ready\n";
        return true;
    }
    return false;
}

void ArduRotorScorpio::PreUpdate(const gz::sim::UpdateInfo &_info,
                                gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused || !dataPtr->ready || !ResolveEntities(_ecm)) return;
    const double now = std::chrono::duration<double>(_info.simTime).count();
    if (now > dataPtr->lastControllerUpdateTime) {
        // Same ordering as the original WorldUpdateBegin callback.
        ReceiveSocketCAN();
        ReceiveMotorCommand();
        if (dataPtr->arduPilotOnline) {
            ApplyMotorForces(now - dataPtr->lastControllerUpdateTime);
            SendState(_info, _ecm);
        }
    }
    dataPtr->lastControllerUpdateTime = now;
}
bool ArduRotorScorpio::InitSocketCAN()
{
#ifdef _WIN32
    return false;
#else
    this->dataPtr->socketcan_fd = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (this->dataPtr->socketcan_fd < 0) {
        return false;
    }

    struct ifreq ifr {};
    strncpy(ifr.ifr_name, this->dataPtr->socketcan_interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(this->dataPtr->socketcan_fd, SIOCGIFINDEX, &ifr) < 0) {
        close(this->dataPtr->socketcan_fd);
        this->dataPtr->socketcan_fd = -1;
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(this->dataPtr->socketcan_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(this->dataPtr->socketcan_fd);
        this->dataPtr->socketcan_fd = -1;
        return false;
    }

    struct can_filter filter {};
    filter.can_id = CAN_EFF_FLAG | (20600U << 8U);
    filter.can_mask = CAN_EFF_FLAG | (0xFFFFU << 8U) | 0x80U;
    if (setsockopt(this->dataPtr->socketcan_fd, SOL_CAN_RAW, CAN_RAW_FILTER,
                   &filter, sizeof(filter)) < 0) {
        close(this->dataPtr->socketcan_fd);
        this->dataPtr->socketcan_fd = -1;
        return false;
    }
    gzlog << "[" << this->dataPtr->modelName << "] SocketCAN legs enabled on ["
          << this->dataPtr->socketcan_interface << "] for com.usl.ServoCmd.\n";
    return true;
#endif
}

/////////////////////////////////////////////////
void ArduRotorScorpio::ReceiveSocketCAN()
{
#ifndef _WIN32
    if (!this->dataPtr->socketcan_enabled || this->dataPtr->socketcan_fd < 0) {
        return;
    }

    struct can_frame frame {};
    while (read(this->dataPtr->socketcan_fd, &frame, sizeof(frame)) == sizeof(frame)) {
        if ((frame.can_id & CAN_EFF_FLAG) == 0 || frame.can_dlc < 1) {
            continue;
        }
        const uint32_t id = frame.can_id & CAN_EFF_MASK;
        if (((id >> 8U) & 0xFFFFU) != 20600U || (id & 0x80U) != 0) {
            continue;
        }
        if (!this->dataPtr->can_frame_logged) {
            gzmsg << "[" << this->dataPtr->modelName
                  << "] received DroneCAN com.usl.ServoCmd frames.\n";
            this->dataPtr->can_frame_logged = true;
        }

        const uint8_t tail = frame.data[frame.can_dlc - 1U];
        const bool start = (tail & 0x80U) != 0;
        const bool end = (tail & 0x40U) != 0;
        const uint8_t toggle = (tail >> 5U) & 1U;
        const uint8_t transfer_id = tail & 0x1FU;
        const uint8_t source_node = id & 0x7FU;

        if (start) {
            this->dataPtr->can_transfer_active = true;
            this->dataPtr->can_transfer_id = transfer_id;
            this->dataPtr->can_source_node = source_node;
            this->dataPtr->can_expected_toggle = toggle ^ 1U;
            this->dataPtr->can_payload_len = 0;
        } else if (!this->dataPtr->can_transfer_active ||
                   transfer_id != this->dataPtr->can_transfer_id ||
                   source_node != this->dataPtr->can_source_node ||
                   toggle != this->dataPtr->can_expected_toggle) {
            this->dataPtr->can_transfer_active = false;
            continue;
        } else {
            this->dataPtr->can_expected_toggle ^= 1U;
        }

        const size_t frame_payload_len = frame.can_dlc - 1U;
        if (this->dataPtr->can_payload_len + frame_payload_len > sizeof(this->dataPtr->can_payload)) {
            this->dataPtr->can_transfer_active = false;
            continue;
        }
        memcpy(this->dataPtr->can_payload + this->dataPtr->can_payload_len,
               frame.data, frame_payload_len);
        this->dataPtr->can_payload_len += frame_payload_len;

        if (!end) {
            continue;
        }

        const size_t crc_bytes = start ? 0U : 2U;
        if (this->dataPtr->can_payload_len <= crc_bytes) {
            this->dataPtr->can_transfer_active = false;
            continue;
        }
        const uint8_t* payload = this->dataPtr->can_payload + crc_bytes;
        const size_t payload_len = this->dataPtr->can_payload_len - crc_bytes;
        const size_t command_count = std::min<size_t>(18U, (payload_len * 8U) / 12U);
        if (command_count != 18U || this->dataPtr->coxa_num != 6 ||
            this->dataPtr->femur_num != 6 || this->dataPtr->tibia_num != 6) {
            this->dataPtr->can_transfer_active = false;
            continue;
        }

        for (size_t command = 0; command < command_count; command++) {
            uint8_t scalar[2] {};
            const size_t first_bit = command * 12U;
            for (size_t bit = 0; bit < 12U; bit++) {
                const size_t source_bit = first_bit + bit;
                const uint8_t value = (payload[source_bit / 8U] >> (7U - source_bit % 8U)) & 1U;
                scalar[bit / 8U] |= value << (7U - bit % 8U);
            }
            scalar[1] >>= 4U;
            const uint16_t pwm = uint16_t(scalar[0]) | (uint16_t(scalar[1]) << 8U);
            const size_t leg = command / 3U;
            switch (command % 3U) {
            case 0: this->dataPtr->coxa_speed[leg] = pwm; break;
            case 1: this->dataPtr->femur_speed[leg] = pwm; break;
            case 2: this->dataPtr->tibia_speed[leg] = pwm; break;
            }
        }
        if (!this->dataPtr->can_command_logged) {
            gzmsg << "[" << this->dataPtr->modelName
                  << "] decoded all 18 SocketCAN leg commands.\n";
            this->dataPtr->can_command_logged = true;
        }
        this->dataPtr->can_transfer_active = false;
    }
#endif
}

/////////////////////////////////////////////////
void ArduRotorScorpio::ResetPIDs()
{
    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        this->dataPtr->coxa_speed[i] = 1500.0;
    }

    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        this->dataPtr->femur_speed[i] = 1500;
    }

    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        this->dataPtr->tibia_speed[i] = 1500;
    }

    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        this->dataPtr->servo_speed[i] = 1500;
    }

    for (uint8_t i = 0; i < MAX_MOTORS; i++) {
        this->dataPtr->motor_speed[i] = 0.0;
    }

    ApplyMotorForces(0);
}

/////////////////////////////////////////////////
bool ArduRotorScorpio::InitArduPilotSockets(sdf::ElementPtr _sdf) const
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
void ArduRotorScorpio::ApplyMotorForces(const double _dt)
{
    // mav_msgs::msg::Actuators actuator_msg;
    // actuator_msg.angular_velocities.clear();
    // actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[1 - 1]);
    // actuator_msg.angular_velocities.push_back(this->dataPtr->motor_speed[2 - 1]);

    // this->dataPtr->motor_pub->publish(actuator_msg);

    mav_msgs::msg::Actuators coxa_msg;
    for (int i = 0; i < this->dataPtr->coxa_num; i++) {
        coxa_msg.angular_velocities.push_back(
            (this->dataPtr->coxa_speed[i] - 1500.0f) / 500.0f * 120.0f / 57.3f);
    }
    this->dataPtr->coxa_pub->publish(coxa_msg);

    mav_msgs::msg::Actuators femur_msg;
    for (int i = 0; i < this->dataPtr->femur_num; i++) {
        femur_msg.angular_velocities.push_back(
            (this->dataPtr->femur_speed[i] - 1500.0f) / 500.0f * 120.0f / 57.3f);
    }
    this->dataPtr->femur_pub->publish(femur_msg);

    mav_msgs::msg::Actuators tibia_msg;
    for (int i = 0; i < this->dataPtr->tibia_num; i++) {
        tibia_msg.angular_velocities.push_back(
            (this->dataPtr->tibia_speed[i] - 1500.0f) / 500.0f * 120.0f / 57.3f);
    }
    this->dataPtr->tibia_pub->publish(tibia_msg);

    mav_msgs::msg::Actuators motor_msg;
    for (int i = 0; i < this->dataPtr->motor_num; i++) {
        motor_msg.angular_velocities.push_back(this->dataPtr->motor_speed[i]);
    }
    this->dataPtr->motor_pub->publish(motor_msg);

    mav_msgs::msg::Actuators servo_msg;
    for (int i = 0; i < this->dataPtr->servo_num; i++) {
        servo_msg.angular_velocities.push_back(
            (this->dataPtr->servo_speed[i] - 1500.0f) / 500.0f * 45.0f / 57.3f);
    }
    this->dataPtr->servo_pub->publish(servo_msg);
}

/////////////////////////////////////////////////
void ArduRotorScorpio::ReceiveMotorCommand()
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
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));
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
        const int required_channels = this->dataPtr->socketcan_enabled
            ? std::max(this->dataPtr->motor_channel_base + this->dataPtr->motor_num,
                       this->dataPtr->servo_channel_base + this->dataPtr->servo_num)
            : this->dataPtr->coxa_num + this->dataPtr->femur_num + this->dataPtr->tibia_num
                + this->dataPtr->motor_num + this->dataPtr->servo_num;
        const ssize_t expectedPktSize = sizeof(pkt.motorSpeed[0]) * required_channels;

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
        //////////////////////////////////////////////////////////////////
        for (unsigned i = 0; !this->dataPtr->socketcan_enabled && i < this->dataPtr->coxa_num; ++i) {
            if (i < MAX_MOTORS) {
                const double cmd = gz::math::clamp(pkt.motorSpeed[i],
                                                         -2.0f,
                                                         2.0f);
                if (cmd == 0) {
                    this->dataPtr->coxa_speed[i] = 1500.0f;
                    continue;
                }

                this->dataPtr->coxa_speed[i] = (cmd - 0.5f) * 1000.0f + 1500.0f;
            } else {
                gzerr << "[" << this->dataPtr->modelName << "] "
                      << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
            }
        }

        //////////////////////////////////////////////////////////////////
        for (unsigned i = 0; !this->dataPtr->socketcan_enabled && i < this->dataPtr->femur_num; ++i) {
            if (i < MAX_MOTORS) {
                const double cmd = gz::math::clamp(pkt.motorSpeed[i + this->dataPtr->coxa_num],
                                                         -2.0f,
                                                         2.0f);

                if (cmd == 0) {
                    this->dataPtr->femur_speed[i] = 1500.0f;
                    continue;
                }

                this->dataPtr->femur_speed[i] = (cmd - 0.5f) * 1000.0f + 1500.0f;
            } else {
                gzerr << "[" << this->dataPtr->modelName << "] "
                      << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
            }
        }

        //////////////////////////////////////////////////////////////////
        for (unsigned i = 0; !this->dataPtr->socketcan_enabled && i < this->dataPtr->tibia_num; ++i) {
            if (i < MAX_MOTORS) {
                const double cmd = gz::math::clamp(pkt.motorSpeed[i + this->dataPtr->coxa_num + this->dataPtr->femur_num],
                                                         -2.0f,
                                                         2.0f);

                if (cmd == 0) {
                    this->dataPtr->tibia_speed[i] = 1500.0f;
                    continue;
                }

                this->dataPtr->tibia_speed[i] = (cmd - 0.5f) * 1000.0f + 1500.0f;
            } else {
                gzerr << "[" << this->dataPtr->modelName << "] "
                      << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
            }
        }

        //////////////////////////////////////////////////////////////////
        for (unsigned i = 0; i < this->dataPtr->motor_num; ++i) {
            if (i < MAX_MOTORS) {
                const unsigned channel = this->dataPtr->socketcan_enabled
                    ? this->dataPtr->motor_channel_base + i
                    : i + this->dataPtr->coxa_num + this->dataPtr->femur_num + this->dataPtr->tibia_num;
                if (channel >= static_cast<unsigned>(recvChannels)) {
                    continue;
                }
                const double cmd = gz::math::clamp(pkt.motorSpeed[channel],
                                                         -1.0f,
                                                         1.0f);

                this->dataPtr->motor_speed[i] = cmd * 1000.0f;
            } else {
                gzerr << "[" << this->dataPtr->modelName << "] "
                      << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
            }
        }

        //////////////////////////////////////////////////////////////////
        for (unsigned i = 0; i < this->dataPtr->servo_num; ++i) {
            if (i < MAX_MOTORS) {
                const unsigned channel = this->dataPtr->socketcan_enabled
                    ? this->dataPtr->servo_channel_base + i
                    : i + this->dataPtr->coxa_num + this->dataPtr->femur_num
                        + this->dataPtr->tibia_num + this->dataPtr->motor_num;
                if (channel >= static_cast<unsigned>(recvChannels)) {
                    continue;
                }
                const double cmd = gz::math::clamp(pkt.motorSpeed[channel],
                                                         -2.0f,
                                                         2.0f);

                this->dataPtr->servo_speed[i] = (cmd - 0.5f) * 1000.0f + 1500.0f;
            } else {
                gzerr << "[" << this->dataPtr->modelName << "] "
                      << "too many motors, skipping [" << i << " > " << MAX_MOTORS << "].\n";
            }
        }
    }
}

/////////////////////////////////////////////////

void ArduRotorScorpio::SendState(const gz::sim::UpdateInfo &_info,
                               const gz::sim::EntityComponentManager &_ecm) const
{
    gz::msgs::IMU imu;
    {
        std::lock_guard<std::mutex> lock(dataPtr->imuMutex);
        if (!dataPtr->imuValid) return;
        imu = dataPtr->imu;
    }
    const auto velocity = gz::sim::Link(dataPtr->baseLink).WorldLinearVelocity(_ecm);
    if (!velocity) return;
    const auto modelPose = gz::sim::worldPose(dataPtr->model.Entity(), _ecm);
    const auto body = modelXYZToAirplaneXForwardZDown + modelPose;
    const auto ned = body - gazeboXYZToNED;
    const auto v = gazeboXYZToNED.Rot().RotateVectorReverse(*velocity);
    fdmPacket packet{};
    packet.timestamp = std::chrono::duration<double>(_info.simTime).count();
    packet.imuAngularVelocityRPY[0] = imu.angular_velocity().x();
    packet.imuAngularVelocityRPY[1] = imu.angular_velocity().y();
    packet.imuAngularVelocityRPY[2] = imu.angular_velocity().z();
    packet.imuLinearAccelerationXYZ[0] = imu.linear_acceleration().x();
    packet.imuLinearAccelerationXYZ[1] = imu.linear_acceleration().y();
    packet.imuLinearAccelerationXYZ[2] = imu.linear_acceleration().z();
    packet.imuOrientationQuat[0] = ned.Rot().W();
    packet.imuOrientationQuat[1] = ned.Rot().X();
    packet.imuOrientationQuat[2] = ned.Rot().Y();
    packet.imuOrientationQuat[3] = ned.Rot().Z();
    packet.positionXYZ[0] = ned.Pos().X();
    packet.positionXYZ[1] = ned.Pos().Y();
    packet.positionXYZ[2] = ned.Pos().Z();
    packet.velocityXYZ[0] = v.X();
    packet.velocityXYZ[1] = v.Y();
    packet.velocityXYZ[2] = v.Z();
    dataPtr->socket_out.Send(&packet, sizeof(packet));
}

void ArduRotorScorpio::Reset(const gz::sim::UpdateInfo &,
                            gz::sim::EntityComponentManager &)
{
    if (dataPtr->ready) ResetPIDs();
    dataPtr->arduPilotOnline = false;
    dataPtr->connectionTimeoutCount = 0;
    dataPtr->lastControllerUpdateTime = 0.0;
    dataPtr->can_transfer_active = false;
    std::lock_guard<std::mutex> lock(dataPtr->imuMutex);
    dataPtr->imuValid = false;
}
}  // namespace gazebo
GZ_ADD_PLUGIN(gazebo::ArduRotorScorpio, gz::sim::System,
              gazebo::ArduRotorScorpio::ISystemConfigure,
              gazebo::ArduRotorScorpio::ISystemPreUpdate,
              gazebo::ArduRotorScorpio::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gazebo::ArduRotorScorpio, "ArduRotorScorpio")
