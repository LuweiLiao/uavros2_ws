#include "iwp_mavros.hpp"
#include "string"

#include <cmath>

using std::placeholders::_1;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    if (args.size() < 5) {
        std::cerr << "Usage: iwp_mavros_node <x> <y> <z> <yaw_deg>\n";
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<rclcpp::Node>("iwp_mavros_node");
    auto local_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local", rclcpp::QoS(10));
    auto global_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/global", rclcpp::QoS(10));
    rclcpp::WallRate loop_rate(100.0);

    while (rclcpp::ok()) {

        geometry_msgs::msg::PoseStamped pos;

        pos.pose.position.x = std::stod(args.at(1));
        pos.pose.position.y = std::stod(args.at(2));
        pos.pose.position.z = std::stod(args.at(3));

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0,
                 (-std::stod(args.at(4)) + 90.0) / 57.3);

        pos.pose.orientation = tf2::toMsg(q);

        local_pub->publish(pos);
        global_pub->publish(pos);

        rclcpp::spin_some(node);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
