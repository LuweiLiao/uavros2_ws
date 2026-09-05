cmake_minimum_required(VERSION 3.8)
project(mav_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Actuators.msg
  msg/AttitudeThrust.msg
  msg/RateThrust.msg
  msg/RollPitchYawrateThrust.msg
  msg/TorqueThrust.msg
  msg/Status.msg
  msg/FilteredSensorData.msg
  msg/GpsWaypoint.msg
  msg/TrajectorySetpoint.msg
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

ament_package()
