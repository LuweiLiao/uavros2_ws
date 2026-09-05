/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <chrono>
#include <fstream>
#include <iostream>

#include <Eigen/Geometry>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

bool sim_running = false;

static const int64_t kNanoSecondsInSecond = 1000000000;

void callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  (void)msg;
  sim_running = true;
}

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waypoint_publisher");

  RCLCPP_INFO(node->get_logger(), "Started waypoint_publisher.");

  const auto args = rclcpp::remove_ros_arguments(argc, argv);

  if (args.size() != 2 && args.size() != 3) {
    RCLCPP_ERROR(node->get_logger(),
        "Usage: waypoint_publisher <waypoint_file>\n"
        "The waypoint file should be structured as: space separated: "
        "wait_time [s] x[m] y[m] z[m] yaw[deg])");
    rclcpp::shutdown();
    return -1;
  }

  std::vector<WaypointWithTime> waypoints;
  const float DEG_2_RAD = M_PI / 180.0;

  std::ifstream wp_file(args.at(1).c_str());

  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    // Only read complete waypoints.
    while (wp_file >> t >> x >> y >> z >> yaw) {
      waypoints.push_back(WaypointWithTime(t, x, y, z, yaw * DEG_2_RAD));
    }
    wp_file.close();
    RCLCPP_INFO(node->get_logger(), "Read %d waypoints.",
                static_cast<int>(waypoints.size()));
  } else {
    RCLCPP_ERROR(node->get_logger(), "Unable to open poses file: %s",
                 args.at(1).c_str());
    rclcpp::shutdown();
    return -1;
  }

  // The IMU is used, to determine if the simulator is running or not.
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, &callback);

  auto wp_pub =
      node->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  RCLCPP_INFO(node->get_logger(), "Wait for simulation to become ready...");

  while (!sim_running && rclcpp::ok()) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(node->get_logger(), "...ok");

  // Wait for 30s such that everything can settle and the mav flies to the initial position.
  rclcpp::sleep_for(std::chrono::seconds(30));

  RCLCPP_INFO(node->get_logger(), "Start publishing waypoints.");

  auto msg = std::make_shared<trajectory_msgs::msg::MultiDOFJointTrajectory>();
  msg->header.stamp = node->get_clock()->now();
  msg->points.resize(waypoints.size());
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  for (size_t i = 0; i < waypoints.size(); ++i) {
    WaypointWithTime& wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSecond);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }
  wp_pub->publish(*msg);

  rclcpp::spin_some(node);
  rclcpp::shutdown();

  return 0;
}
