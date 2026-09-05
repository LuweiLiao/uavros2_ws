cmake_minimum_required(VERSION 3.8)
project(rotors_hil_interface)

find_package(ament_cmake REQUIRED)

# Match the ROS 1 package's optional dependency contract.  The original
# CMakeLists.txt used `find_package(Mavlink QUIET)` and exposed an empty valid
# package when MAVLink was not installed; it did not make HIL a reason to hide
# or rename the package.  Keep that behavior in ROS 2 while allowing the full
# target to build whenever the official mavlink package is sourced.
find_package(mavlink QUIET CONFIG)
if(NOT mavlink_FOUND)
  message(STATUS "MAVLink not found; keeping rotors_hil_interface as an empty package, as in ROS 1.")
  ament_package()
  return()
endif()

find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# The ROS 2 mav_msgs package installs its generated headers below a nested
# prefix while retaining the original public helper headers (including
# default_topics.h) one level above that tree.
set(MAV_MSGS_INCLUDE_PREFIX "${mav_msgs_DIR}/../../../include")

add_library(rotors_hil_interface SHARED
  src/hil_sensor_level_interface.cpp
  src/hil_state_level_interface.cpp
)
target_compile_features(rotors_hil_interface PUBLIC cxx_std_17)
target_include_directories(rotors_hil_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${mavlink_INCLUDE_DIRS}
    ${MAV_MSGS_INCLUDE_PREFIX}
)
target_link_libraries(rotors_hil_interface Eigen3::Eigen)
ament_target_dependencies(rotors_hil_interface
  geometry_msgs
  mav_msgs
  mavros_msgs
  rclcpp
  sensor_msgs
)

add_executable(rotors_hil_interface_node src/hil_interface_node.cpp)
target_compile_features(rotors_hil_interface_node PRIVATE cxx_std_17)
target_link_libraries(rotors_hil_interface_node rotors_hil_interface)
ament_target_dependencies(rotors_hil_interface_node
  geometry_msgs
  mav_msgs
  mavros_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  rotors_hil_interface
  rotors_hil_interface_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  mav_msgs
  mavros_msgs
  rclcpp
  sensor_msgs
)

ament_package()
