cmake_minimum_required(VERSION 3.8)
project(vrpn_mavros)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

install(PROGRAMS
  scripts/vrpn_flu_to_enu.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclpy geometry_msgs)
ament_package()
