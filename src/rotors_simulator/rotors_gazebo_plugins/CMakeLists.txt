cmake_minimum_required(VERSION 3.8)
project(rotors_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
gz_find_package(gz-msgs10 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(rotors_comm REQUIRED)

# Restore the original protobuf target and schemas in place. Only imported
# Gazebo message APIs change from gazebo.msgs to gz.msgs; field IDs stay fixed.
find_path(GAZEBO_SIM_PROTO_DIR gz/msgs/pose.proto
  HINTS "${gz-msgs10_DIR}/../../../share/gz/gz-msgs10/protos" REQUIRED)
set(Protobuf_IMPORT_DIRS "${GAZEBO_SIM_PROTO_DIR}")
file(GLOB msgs msgs/*.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(mav_msgs SHARED ${PROTO_SRCS})
target_include_directories(mav_msgs PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(mav_msgs PUBLIC protobuf::libprotobuf gz-msgs10::gz-msgs10)

add_library(rotors_gazebo_ros_interface_plugin SHARED src/gazebo_ros_interface_plugin.cpp)
target_compile_features(rotors_gazebo_ros_interface_plugin PRIVATE cxx_std_17)
target_include_directories(rotors_gazebo_ros_interface_plugin PRIVATE include)
target_link_libraries(rotors_gazebo_ros_interface_plugin
  mav_msgs gz-sim8::gz-sim8 gz-transport13::gz-transport13)
ament_target_dependencies(rotors_gazebo_ros_interface_plugin
  rclcpp tf2_ros std_msgs sensor_msgs geometry_msgs nav_msgs mav_msgs rotors_comm)
set_target_properties(rotors_gazebo_ros_interface_plugin PROPERTIES INSTALL_RPATH "$ORIGIN")

# The source file and target retain the original RotorS names.  Only the
# Gazebo API implementations below have been ported; the remaining original
# plugins stay in this same package and are
# migrated one at a time.  Do not substitute a new vehicle-specific aero
# target for this library.
add_library(rotors_gazebo_motor_model SHARED
  src/gazebo_motor_model.cpp
)
target_compile_features(rotors_gazebo_motor_model PRIVATE cxx_std_17)
target_include_directories(rotors_gazebo_motor_model PRIVATE include)
target_link_libraries(rotors_gazebo_motor_model PRIVATE
  mav_msgs
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)
set_target_properties(rotors_gazebo_motor_model PROPERTIES INSTALL_RPATH "$ORIGIN")

add_library(rotors_gazebo_imu_plugin SHARED src/gazebo_imu_plugin.cpp)
target_compile_features(rotors_gazebo_imu_plugin PRIVATE cxx_std_17)
target_include_directories(rotors_gazebo_imu_plugin PRIVATE include)
target_link_libraries(rotors_gazebo_imu_plugin PRIVATE
  mav_msgs Eigen3::Eigen gz-sim8::gz-sim8 gz-transport13::gz-transport13)
set_target_properties(rotors_gazebo_imu_plugin PROPERTIES INSTALL_RPATH "$ORIGIN")

install(TARGETS mav_msgs rotors_gazebo_ros_interface_plugin rotors_gazebo_motor_model
  rotors_gazebo_imu_plugin
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(FILES ${PROTO_HDRS} DESTINATION include)

ament_package()
