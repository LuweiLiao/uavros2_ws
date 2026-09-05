cmake_minimum_required(VERSION 3.8)
project(rotors_joy_interface)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(rotors_joy_interface src/joy.cpp)
target_compile_features(rotors_joy_interface PRIVATE cxx_std_17)
target_include_directories(rotors_joy_interface PRIVATE include)
# rosidl packages export their generated headers below include/mav_msgs while
# the original public helper headers keep the <mav_msgs/...> include contract.
# Add the install prefix itself so both layouts resolve when this package is
# built against a workspace-local mav_msgs.
target_include_directories(rotors_joy_interface PRIVATE
  ${mav_msgs_DIR}/../../../include)
ament_target_dependencies(rotors_joy_interface
  geometry_msgs mav_msgs rclcpp sensor_msgs trajectory_msgs)

install(TARGETS rotors_joy_interface
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY media/ DESTINATION share/${PROJECT_NAME}/media)
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME})

ament_package()
