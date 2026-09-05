#include "GazeboAdm002Plugin.hh"

#include "Adm002Protocol.hh"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>

#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/WrenchMeasured.hh>
#include <gz/sim/Util.hh>

namespace gazebo
{

struct GazeboAdm002Plugin::sockaddr_in_storage {
    sockaddr_in value{};
};

GazeboAdm002Plugin::GazeboAdm002Plugin() = default;

GazeboAdm002Plugin::~GazeboAdm002Plugin()
{
    CloseSocket();
}

void GazeboAdm002Plugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &/*ecm*/,
    gz::sim::EventManager &/*event_manager*/)
{
    sensor_entity_ = entity;

    force_axis_ = sdf->Get("forceAxis", force_axis_).first;
    force_sign_ = sdf->Get("forceSign", force_sign_).first;
    force_scale_ = sdf->Get("forceScale", force_scale_).first;
    ros_topic_ = sdf->Get("rosTopic", ros_topic_).first;
    udp_bind_address_ = sdf->Get("udpBindAddress", udp_bind_address_).first;
    udp_bind_port_ = static_cast<uint16_t>(sdf->Get(
        "udpBindPort", static_cast<uint32_t>(udp_bind_port_)).first);
    stream_rate_hz_ = sdf->Get("streamRateHz", stream_rate_hz_).first;
    device_address_ = static_cast<uint8_t>(sdf->Get(
        "deviceAddress", static_cast<uint32_t>(device_address_)).first);

    if (force_axis_ != "x" && force_axis_ != "y" && force_axis_ != "z") {
        gzerr << "GazeboAdm002Plugin forceAxis must be x, y or z\n";
        return;
    }
    if (stream_rate_hz_ <= 0.0) {
        gzerr << "GazeboAdm002Plugin streamRateHz must be positive\n";
        return;
    }
    if (!OpenSocket()) {
        gzerr << "GazeboAdm002Plugin failed to bind " << udp_bind_address_
              << ":" << udp_bind_port_ << "\n";
        return;
    }

    // Use a private ROS 2 context so the Gazebo server does not need a
    // process-wide rclcpp::init() call and the plugin can shut down cleanly.
    ros_context_ = std::make_shared<rclcpp::Context>();
    ros_context_->init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.context(ros_context_);
    ros_node_ = std::make_shared<rclcpp::Node>(
        "gazebo_adm002_plugin", node_options);
    force_publisher_ = ros_node_->create_publisher<std_msgs::msg::Float64>(
        ros_topic_, rclcpp::QoS(10));

    peer_address_.reset(new sockaddr_in_storage());
    last_stream_time_s_ = 0.0;

    gzmsg << "Gazebo ADM002 sensor entity:" << sensor_entity_
          << " axis:" << force_axis_ << " sign:" << force_sign_
          << " UDP:" << udp_bind_address_ << ":" << udp_bind_port_
          << " stream_rate_hz:" << stream_rate_hz_ << "\n";
}

void GazeboAdm002Plugin::PostUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm)
{
    OnUpdate(info, ecm);
}

void GazeboAdm002Plugin::Reset(
    const gz::sim::UpdateInfo &/*info*/,
    gz::sim::EntityComponentManager &/*ecm*/)
{
    stream_enabled_ = false;
    peer_address_.reset(new sockaddr_in_storage());
    last_stream_time_s_ = 0.0;
}

bool GazeboAdm002Plugin::OpenSocket()
{
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        return false;
    }
    const int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    fcntl(socket_fd_, F_SETFL, fcntl(socket_fd_, F_GETFL, 0) | O_NONBLOCK);

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(udp_bind_port_);
    if (inet_pton(AF_INET, udp_bind_address_.c_str(), &address.sin_addr) != 1 ||
        bind(socket_fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        CloseSocket();
        return false;
    }
    return true;
}

void GazeboAdm002Plugin::CloseSocket()
{
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

double GazeboAdm002Plugin::SelectedForceNewton(
    const gz::sim::EntityComponentManager &ecm) const
{
    const auto *wrench = ecm.Component<gz::sim::components::WrenchMeasured>(
        sensor_entity_);
    if (!wrench) {
        return 0.0;
    }

    const auto &force = wrench->Data().force();
    const double component = force_axis_ == "x" ? force.x() :
                             force_axis_ == "y" ? force.y() : force.z();
    return component * force_sign_;
}

void GazeboAdm002Plugin::OnUpdate(
    const gz::sim::UpdateInfo &info,
    const gz::sim::EntityComponentManager &ecm)
{
    if (info.paused) {
        return;
    }

    const double force_n = SelectedForceNewton(ecm);
    if (force_publisher_) {
        std_msgs::msg::Float64 force_message;
        force_message.data = force_n;
        force_publisher_->publish(force_message);
    }

    std::array<uint8_t, 256> request{};
    while (true) {
        sockaddr_in source{};
        socklen_t source_length = sizeof(source);
        const ssize_t received = recvfrom(socket_fd_, request.data(), request.size(), 0,
            reinterpret_cast<sockaddr*>(&source), &source_length);
        if (received < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                gzerr << "Gazebo ADM002 receive failed: "
                      << std::strerror(errno) << "\n";
            }
            break;
        }
        if (!uav_gazebo_plugin::Adm002Protocol::IsEnableStreamCommand(
                request.data(), static_cast<std::size_t>(received), device_address_)) {
            continue;
        }
        peer_address_->value = source;
        stream_enabled_ = true;
        const auto ack = uav_gazebo_plugin::Adm002Protocol::EncodeEnableAck(device_address_);
        sendto(socket_fd_, ack.data(), ack.size(), 0,
            reinterpret_cast<const sockaddr*>(&peer_address_->value), sizeof(peer_address_->value));
    }

    if (!stream_enabled_) {
        return;
    }

    const double now_s = std::chrono::duration<double>(info.simTime).count();
    if (now_s < last_stream_time_s_) {
        last_stream_time_s_ = now_s;
    }
    const double period_s = 1.0 / stream_rate_hz_;
    if (now_s - last_stream_time_s_ < period_s) {
        return;
    }
    last_stream_time_s_ = now_s;

    const auto frame = uav_gazebo_plugin::Adm002Protocol::EncodeForceNewton(force_n, force_scale_);
    sendto(socket_fd_, frame.data(), frame.size(), 0,
        reinterpret_cast<const sockaddr*>(&peer_address_->value), sizeof(peer_address_->value));
}

}  // namespace gazebo

GZ_ADD_PLUGIN(
    gazebo::GazeboAdm002Plugin,
    gz::sim::System,
    gazebo::GazeboAdm002Plugin::ISystemConfigure,
    gazebo::GazeboAdm002Plugin::ISystemPostUpdate,
    gazebo::GazeboAdm002Plugin::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gazebo::GazeboAdm002Plugin, "GazeboAdm002Plugin")
// Gazebo Sim resolves the SDF <plugin name> in addition to the library name.
// Preserve the original tilt_quadcopter_fix instance name verbatim.
GZ_ADD_PLUGIN_ALIAS(gazebo::GazeboAdm002Plugin, "front_adm002")
