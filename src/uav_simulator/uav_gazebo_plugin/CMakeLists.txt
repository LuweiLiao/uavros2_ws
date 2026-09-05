cmake_minimum_required(VERSION 3.8)

project(uav_gazebo_plugin)

# This package keeps the ROS 1 source tree and target names.  Gazebo Sim
# ports are added under their original source and library names; no replacement
# vehicle-specific aero plugin or package boundary is introduced.
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-sim8 REQUIRED)
gz_find_package(gz-transport13 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(HelloWorldPlugin SHARED
  src/HelloWorldPlugin.cc
)
set_target_properties(HelloWorldPlugin PROPERTIES OUTPUT_NAME HelloWorldPlugin)
target_compile_features(HelloWorldPlugin PRIVATE cxx_std_17)
target_include_directories(HelloWorldPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(HelloWorldPlugin gz-sim8::gz-sim8)

add_library(HelloModelPlugin SHARED
  src/HelloModelPlugin.cc
)
set_target_properties(HelloModelPlugin PROPERTIES OUTPUT_NAME HelloModelPlugin)
target_compile_features(HelloModelPlugin PRIVATE cxx_std_17)
target_include_directories(HelloModelPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(HelloModelPlugin gz-sim8::gz-sim8)

add_library(MotorsPlugin SHARED
  src/MotorsPlugin.cc
)
set_target_properties(MotorsPlugin PROPERTIES OUTPUT_NAME MotorsPlugin)
target_compile_features(MotorsPlugin PRIVATE cxx_std_17)
target_include_directories(MotorsPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(MotorsPlugin gz-sim8::gz-sim8)

add_library(ArduRotorNormPlugin SHARED
  src/ArduRotorNormPlugin.cc
)
target_compile_features(ArduRotorNormPlugin PRIVATE cxx_std_17)
target_include_directories(ArduRotorNormPlugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ArduRotorNormPlugin PRIVATE
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)

# The ROS 1 QuadOct source registers the Norm class under this historical
# target name.  Keep the same shared-library filename and SDF loader contract;
# the Norm port already emits the original eight-channel order for motor_num=8.
add_library(ArduRotorQuadOctPlugin SHARED
  src/ArduRotorQuadOctPlugin.cc
)
set_target_properties(ArduRotorQuadOctPlugin PROPERTIES
  OUTPUT_NAME ArduRotorQuadOctPlugin
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)
target_compile_features(ArduRotorQuadOctPlugin PRIVATE cxx_std_17)
target_include_directories(ArduRotorQuadOctPlugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ArduRotorQuadOctPlugin
  PRIVATE
    ArduRotorNormPlugin
    gz-sim8::gz-sim8
    gz-plugin2::gz-plugin2
)

add_library(ArduRotorTiltQuadcopter SHARED src/ArduRotorTiltQuadcopter.cc)
target_compile_features(ArduRotorTiltQuadcopter PRIVATE cxx_std_17)
target_include_directories(ArduRotorTiltQuadcopter PRIVATE include ${mavlink_INCLUDE_DIRS})
target_link_libraries(ArduRotorTiltQuadcopter PRIVATE
  gz-sim8::gz-sim8 gz-transport13::gz-transport13)

add_library(GazeboAdm002Plugin SHARED
  src/GazeboAdm002Plugin.cc
)
set_target_properties(GazeboAdm002Plugin PROPERTIES
  OUTPUT_NAME gazebo_adm002_plugin
)
target_compile_features(GazeboAdm002Plugin PRIVATE cxx_std_17)
target_include_directories(GazeboAdm002Plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(GazeboAdm002Plugin
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
)
ament_target_dependencies(GazeboAdm002Plugin
  rclcpp
  std_msgs
)

add_library(PoleHoldReleasePlugin SHARED
  src/PoleHoldReleasePlugin.cc
)
set_target_properties(PoleHoldReleasePlugin PROPERTIES
  OUTPUT_NAME PoleHoldReleasePlugin
)
target_compile_features(PoleHoldReleasePlugin PRIVATE cxx_std_17)
target_include_directories(PoleHoldReleasePlugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(PoleHoldReleasePlugin
  gz-sim8::gz-sim8
)
ament_target_dependencies(PoleHoldReleasePlugin
  geometry_msgs
  rclcpp
  std_msgs
)

add_library(PoleMavlinkPosePlugin SHARED
  src/PoleMavlinkPosePlugin.cc
)
set_target_properties(PoleMavlinkPosePlugin PROPERTIES
  OUTPUT_NAME PoleMavlinkPosePlugin
)
target_compile_features(PoleMavlinkPosePlugin PRIVATE cxx_std_17)
target_include_directories(PoleMavlinkPosePlugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${mavlink_INCLUDE_DIRS}
)
target_link_libraries(PoleMavlinkPosePlugin
  gz-sim8::gz-sim8
)
ament_target_dependencies(PoleMavlinkPosePlugin
  geometry_msgs
  rclcpp
)

add_library(position_controller SHARED
  src/position_controller.cc
)
set_target_properties(position_controller PROPERTIES
  OUTPUT_NAME position_controller
)
target_compile_features(position_controller PRIVATE cxx_std_17)
target_include_directories(position_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(position_controller
  gz-sim8::gz-sim8
)
ament_target_dependencies(position_controller
  rclcpp
  std_msgs
)

ament_add_gtest(test_adm002_protocol test/test_adm002_protocol.cpp)
target_include_directories(test_adm002_protocol PRIVATE include)

install(TARGETS HelloWorldPlugin HelloModelPlugin MotorsPlugin ArduRotorNormPlugin
  ArduRotorQuadOctPlugin ArduRotorTiltQuadcopter
  GazeboAdm002Plugin PoleHoldReleasePlugin PoleMavlinkPosePlugin
  position_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_package()
