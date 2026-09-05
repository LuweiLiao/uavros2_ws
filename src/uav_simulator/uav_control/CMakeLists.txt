cmake_minimum_required(VERSION 3.8)

project(uav_control)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gz-cmake3 REQUIRED)
gz_find_package(gz-msgs10 REQUIRED)
gz_find_package(gz-transport13 REQUIRED)

add_library(pid_control
  src/pid_control.cpp
)
target_compile_features(pid_control PUBLIC cxx_std_17)
target_include_directories(pid_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(vectortricopter
  src/vectortricopter/att_controller.cc
  src/vectortricopter/control_allocator.cc
  src/vectortricopter/pos_controller.cc
  src/vectortricopter/rate_controller.cc
  src/vectortricopter/scheduler.cc
  src/vectortricopter/vehicle.cc
  src/vectortricopter/world.cc
)
target_compile_features(vectortricopter PUBLIC cxx_std_17)
target_include_directories(vectortricopter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vectortricopter Eigen3::Eigen)
ament_target_dependencies(vectortricopter
  mav_msgs nav_msgs rclcpp sensor_msgs)

add_executable(tricopter_control_node src/tricopter_control_node.cc)
target_compile_features(tricopter_control_node PRIVATE cxx_std_17)
target_include_directories(tricopter_control_node PRIVATE include)
target_link_libraries(tricopter_control_node vectortricopter)
ament_target_dependencies(tricopter_control_node mav_msgs rclcpp)

add_executable(uav_pid_control_node src/uav_pid_control_node.cpp)
target_compile_features(uav_pid_control_node PRIVATE cxx_std_17)
target_include_directories(uav_pid_control_node PRIVATE include)
target_link_libraries(uav_pid_control_node pid_control Eigen3::Eigen)
ament_target_dependencies(uav_pid_control_node
  mav_msgs nav_msgs rclcpp sensor_msgs std_msgs)

add_executable(iwp_claw_node src/iwp_claw_node.cpp)
target_compile_features(iwp_claw_node PRIVATE cxx_std_17)
target_include_directories(iwp_claw_node PRIVATE include)
target_link_libraries(iwp_claw_node
  Eigen3::Eigen
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13)
ament_target_dependencies(iwp_claw_node rclcpp)

add_executable(iwp_mavros_node src/iwp_mavros_node.cpp)
target_compile_features(iwp_mavros_node PRIVATE cxx_std_17)
target_include_directories(iwp_mavros_node PRIVATE include)
ament_target_dependencies(iwp_mavros_node
  geometry_msgs rclcpp tf2 tf2_geometry_msgs)

install(TARGETS
  pid_control
  vectortricopter
  tricopter_control_node
  uav_pid_control_node
  iwp_claw_node
  iwp_mavros_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()
