#pragma once

#include <chrono>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include "iostream"
#include "string"
#include <sstream>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include <gz/msgs/double.pb.h>
#include <gz/transport/Node.hh>

#include <math.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class SocketTCP {
private:
    int sockfd{-1};

    const char* ip_addr = "127.0.0.1";

    const int port = 5762;

    struct sockaddr_in servaddr;

    const int maxsize = 4096;

public:
    void init()
    {
        if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
            exit(0);
        }

        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_addr.s_addr = inet_addr(ip_addr);
        servaddr.sin_port = htons(port);

        if (connect(sockfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
            printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
            exit(0);
        }
    }

    int getdata(char* buff) { return recv(sockfd, buff, maxsize, 0); }

    ~SocketTCP()
    {
        if (sockfd >= 0) close(sockfd);
    }
};

class ParseData {
private:
    const uint8_t CLAW_CLOSE = 0xAE;
    const uint8_t CLAW_OPEN = 0xBE;
    const uint8_t CLAW_STOP = 0xCE;

    uint8_t buffer[50];

    int recv_claw_close;

    uint8_t _data_len;
    uint8_t _data_cnt;
    uint8_t state;

public:
    ParseData()
    {
        _data_len = 0;

        _data_cnt = 0;

        state = 0;

        recv_claw_close = 0;
    }

    void recv_all(uint8_t* data_buf, uint8_t num)
    {
        uint8_t sum = 0;
        for (uint8_t i = 0; i < (num - 1); i++)
            sum += *(data_buf + i);

        if (!(sum == *(data_buf + num - 1)))
            return; //??sum

        if (!(*(data_buf) == 0xAA && *(data_buf + 1) == 0xAA))
            return; //????

        if (*(data_buf + 4) == CLAW_CLOSE) {
            recv_claw_close = 1;
        } else if (*(data_buf + 4) == CLAW_STOP) {
            recv_claw_close = 0;
        } else if (*(data_buf + 4) == CLAW_OPEN) {
            recv_claw_close = -1;
        }
    }

    void recv_data(uint8_t data)
    {
        if (state == 0 && data == 0xAA) {
            state = 1;
            buffer[0] = data;
        } else if (state == 1 && data == 0xAA) {
            state = 2;
            buffer[1] = data;
        } else if (state == 2 && data < 0XF1) {
            state = 3;
            buffer[2] = data;
        } else if (state == 3 && data < 50) {
            state = 4;
            buffer[3] = data;
            _data_len = data;
            _data_cnt = 0;
        } else if (state == 4 && _data_len > 0) {
            _data_len--;
            buffer[4 + _data_cnt++] = data;
            if (_data_len == 0)
                state = 5;
        } else if (state == 5) {
            state = 0;
            buffer[4 + _data_cnt] = data;
            recv_all(buffer, _data_cnt + 5);
        } else
            state = 0;
    }

    int getState()
    {
        printf("recv_claw_close=%d\r\n", recv_claw_close);

        return recv_claw_close;
    }
};

class IWP_CLAW_NODE {
private:
    rclcpp::Node::SharedPtr nh;

    rclcpp::Node::SharedPtr private_nh;

    // Gazebo Classic exposed ApplyJointEffort as a ROS service.  Gazebo Sim's
    // official ApplyJointForce system accepts gz.msgs.Double on one command
    // topic per joint; only this transport boundary changes.
    gz::transport::Node gz_node;
    gz::transport::Node::Publisher claw_publishers[2];

    SocketTCP socket;

    ParseData parsedata;

public:
    IWP_CLAW_NODE();

    IWP_CLAW_NODE(const rclcpp::Node::SharedPtr& _nh,
                  const rclcpp::Node::SharedPtr& _private_nh);

    ~IWP_CLAW_NODE() = default;

    void open_claw();

    void close_claw();

    void apply_joint_effort(const char* joint_name, double effort);

    void update();
};

IWP_CLAW_NODE::IWP_CLAW_NODE(
    const rclcpp::Node::SharedPtr& _nh,
    const rclcpp::Node::SharedPtr& _private_nh)
    : nh(_nh)
    , private_nh(_private_nh)
{
    socket.init();
}

void IWP_CLAW_NODE::apply_joint_effort(const char* joint_name, double effort)
{
    if (joint_name == nullptr) {
        return;
    }

    // Keep the ROS 1 fully scoped joint names.  ApplyJointForce uses the
    // scoped name below the model command topic.
    const std::string scoped_name(joint_name);
    const std::size_t model_separator = scoped_name.find("::");
    if (model_separator == std::string::npos) {
        RCLCPP_WARN(nh->get_logger(), "Invalid scoped Gazebo joint name: %s",
                    joint_name);
        return;
    }

    const std::string model_name = scoped_name.substr(0, model_separator);
    const std::string joint_name_in_model =
        scoped_name.substr(model_separator + 2);
    const std::string topic = "/model/" + model_name + "/joint/" +
                              joint_name_in_model + "/cmd_force";

    int publisher_index = -1;
    if (scoped_name.find("usl_iwp_claw_0::") != std::string::npos) {
        publisher_index = 0;
    } else if (scoped_name.find("usl_iwp_claw_1::") != std::string::npos) {
        publisher_index = 1;
    }

    if (publisher_index < 0) {
        RCLCPP_WARN(nh->get_logger(), "Unsupported Gazebo claw joint: %s",
                    joint_name);
        return;
    }

    if (!claw_publishers[publisher_index]) {
        claw_publishers[publisher_index] =
            gz_node.Advertise<gz::msgs::Double>(topic);
    }

    gz::msgs::Double command;
    command.set_data(effort);
    claw_publishers[publisher_index].Publish(command);

    // ApplyJointForce keeps the last command.  Reproduce the ROS 1 service's
    // one-second duration by sending the official zero-force command later.
    const auto publisher = claw_publishers[publisher_index];
    std::thread([publisher]() {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        gz::msgs::Double stop;
        stop.set_data(0.0);
        auto delayed_publisher = publisher;
        delayed_publisher.Publish(stop);
    }).detach();
}

void IWP_CLAW_NODE::open_claw()
{
    apply_joint_effort(
        "usl_iwp::usl_iwp_claw_0::usl_iwp_claw_flange_joint", 200);
    apply_joint_effort(
        "usl_iwp::usl_iwp_claw_1::usl_iwp_claw_flange_joint", 200);
}

void IWP_CLAW_NODE::close_claw()
{
    apply_joint_effort(
        "usl_iwp::usl_iwp_claw_0::usl_iwp_claw_flange_joint", -200);
    apply_joint_effort(
        "usl_iwp::usl_iwp_claw_1::usl_iwp_claw_flange_joint", -200);
}

void IWP_CLAW_NODE::update()
{
    char buff[4096];

    int len = socket.getdata(buff);

    if (len > 0) {
        for (int i = 0; i < len; i++) {
            parsedata.recv_data(static_cast<uint8_t>(buff[i]));
        }
    }
            
        if (parsedata.getState() == 1) {
            close_claw();
        } else if (parsedata.getState() == -1) {
            open_claw();
        }
}
