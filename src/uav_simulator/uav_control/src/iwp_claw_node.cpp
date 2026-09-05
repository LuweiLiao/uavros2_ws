#include "iwp_claw_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto nh = std::make_shared<rclcpp::Node>("iwp_claw_node");
    auto private_nh = nh;

    IWP_CLAW_NODE iwp_claw_node(nh, private_nh);

    rclcpp::WallRate loop_rate(100.0);

    while (rclcpp::ok()) {

        iwp_claw_node.update();

        rclcpp::spin_some(nh);

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
