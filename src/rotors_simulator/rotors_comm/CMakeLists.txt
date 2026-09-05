cmake_minimum_required(VERSION 3.8)
project(rotors_comm)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/WindSpeed.msg
  srv/Octomap.srv
  srv/RecordRosbag.srv
  DEPENDENCIES builtin_interfaces geometry_msgs octomap_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
