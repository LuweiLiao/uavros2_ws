cmake_minimum_required(VERSION 3.8)
project(rotors_control)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# rosidl exports the generated headers below include/mav_msgs.  The original
# public helper headers (conversions.h, eigen_mav_msgs.h, ...) retain their
# <mav_msgs/...> include contract and are installed one level above that
# generated tree, so add the package include prefix explicitly.
set(MAV_MSGS_INCLUDE_PREFIX "${mav_msgs_DIR}/../../../include")

add_library(lee_position_controller SHARED
  src/library/lee_position_controller.cpp
)
target_compile_features(lee_position_controller PUBLIC cxx_std_11)
target_include_directories(lee_position_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MAV_MSGS_INCLUDE_PREFIX}
)
target_link_libraries(lee_position_controller Eigen3::Eigen)
ament_target_dependencies(lee_position_controller
  geometry_msgs mav_msgs nav_msgs sensor_msgs trajectory_msgs)

add_library(roll_pitch_yawrate_thrust_controller SHARED
  src/library/roll_pitch_yawrate_thrust_controller.cpp
)
target_compile_features(roll_pitch_yawrate_thrust_controller PUBLIC cxx_std_11)
target_include_directories(roll_pitch_yawrate_thrust_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MAV_MSGS_INCLUDE_PREFIX}
)
target_link_libraries(roll_pitch_yawrate_thrust_controller Eigen3::Eigen)
ament_target_dependencies(roll_pitch_yawrate_thrust_controller
  geometry_msgs mav_msgs nav_msgs sensor_msgs trajectory_msgs)

add_executable(lee_position_controller_node
  src/nodes/lee_position_controller_node.cpp)
target_compile_features(lee_position_controller_node PRIVATE cxx_std_17)
target_link_libraries(lee_position_controller_node lee_position_controller)
ament_target_dependencies(lee_position_controller_node
  geometry_msgs mav_msgs nav_msgs rclcpp sensor_msgs trajectory_msgs)

add_executable(roll_pitch_yawrate_thrust_controller_node
  src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
target_compile_features(roll_pitch_yawrate_thrust_controller_node PRIVATE cxx_std_17)
target_link_libraries(roll_pitch_yawrate_thrust_controller_node
  roll_pitch_yawrate_thrust_controller)
ament_target_dependencies(roll_pitch_yawrate_thrust_controller_node
  geometry_msgs mav_msgs nav_msgs rclcpp sensor_msgs trajectory_msgs)

install(TARGETS
  lee_position_controller
  roll_pitch_yawrate_thrust_controller
  lee_position_controller_node
  roll_pitch_yawrate_thrust_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs mav_msgs nav_msgs rclcpp sensor_msgs trajectory_msgs)

ament_package()
