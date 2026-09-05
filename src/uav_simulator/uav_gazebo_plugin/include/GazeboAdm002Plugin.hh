#ifndef UAV_GAZEBO_PLUGIN_GAZEBO_ADM002_PLUGIN_HH_
#define UAV_GAZEBO_PLUGIN_GAZEBO_ADM002_PLUGIN_HH_

#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sdf/sdf.hh>

#include <cstdint>
#include <memory>
#include <string>

namespace gazebo
{

/// Gazebo Sim port of the ROS 1 GazeboAdm002Plugin.
///
/// The plugin remains in the original package/source file and keeps the
/// original SDF keys, UDP wire protocol, output library name and ROS topic.
/// Only the Gazebo sensor and ROS client-library boundaries are adapted.
class GazeboAdm002Plugin final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemReset
{
public:
    GazeboAdm002Plugin();
    ~GazeboAdm002Plugin() override;

    void Configure(
        const gz::sim::Entity &entity,
        const std::shared_ptr<const sdf::Element> &sdf,
        gz::sim::EntityComponentManager &ecm,
        gz::sim::EventManager &event_manager) override;

    void PostUpdate(
        const gz::sim::UpdateInfo &info,
        const gz::sim::EntityComponentManager &ecm) override;

    void Reset(
        const gz::sim::UpdateInfo &info,
        gz::sim::EntityComponentManager &ecm) override;

private:
    void OnUpdate(const gz::sim::UpdateInfo &info,
                  const gz::sim::EntityComponentManager &ecm);
    bool OpenSocket();
    void CloseSocket();
    double SelectedForceNewton(
        const gz::sim::EntityComponentManager &ecm) const;

    gz::sim::Entity sensor_entity_{gz::sim::kNullEntity};
    std::shared_ptr<rclcpp::Context> ros_context_;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_publisher_;

    std::string force_axis_ = "x";
    double force_sign_ = 1.0;
    double force_scale_ = 1.0;
    std::string ros_topic_ = "/tilt_quadcopter_fix/front_rod/contact_force_x";
    std::string udp_bind_address_ = "127.0.0.1";
    uint16_t udp_bind_port_ = 9024;
    double stream_rate_hz_ = 100.0;
    uint8_t device_address_ = 1;

    int socket_fd_ = -1;
    bool stream_enabled_ = false;
    struct sockaddr_in_storage;
    std::unique_ptr<sockaddr_in_storage> peer_address_;
    double last_stream_time_s_ = 0.0;
};

}  // namespace gazebo

#endif  // UAV_GAZEBO_PLUGIN_GAZEBO_ADM002_PLUGIN_HH_
