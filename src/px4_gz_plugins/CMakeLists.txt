cmake_minimum_required(VERSION 3.16)
project(px4_gz_plugins LANGUAGES C CXX)

# Also supports a standalone CMake build, without a ROS installation.
find_package(ament_cmake QUIET)
# Jazzy installs Gazebo through vendor packages; standalone builds use system libs.
find_package(gz_sim_vendor QUIET)
find_package(gz_transport_vendor QUIET)
find_package(gz_sensors_vendor QUIET)
find_package(gz_plugin_vendor QUIET)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(Protobuf REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
set(CMAKE_BUILD_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH "$ORIGIN")
set(GZ_SIM_TARGET gz-sim8::gz-sim8)
set(GZ_TRANSPORT_TARGET gz-transport13::gz-transport13)
set(GZ_SENSORS_TARGET gz-sensors8::gz-sensors8)
set(GZ_PLUGIN_TARGET gz-plugin2::gz-plugin2)
add_subdirectory(msgs)

# Source is vendored, including OpticalFlow's klt dependency: no build-time fetch.
foreach(plugin optical_flow gstreamer moving_platform_controller generic_motor
               buoyancy spacecraft_thruster motor_failure airspeed)
  add_subdirectory(upstream/${plugin})
endforeach()
install(TARGETS OpticalFlowSystem OpticalFlow MovingPlatformController
  GenericMotorModelPlugin BuoyancySystemPlugin SpacecraftThrusterModelPlugin
  MotorFailurePlugin AirSpeedPlugin LIBRARY DESTINATION lib/px4_gz_plugins)
if(TARGET GstCameraSystem)
  install(TARGETS GstCameraSystem LIBRARY DESTINATION lib/px4_gz_plugins)
endif()
install(FILES LICENSE README.md PROVENANCE.json DESTINATION share/${PROJECT_NAME})
if(ament_cmake_FOUND)
  ament_environment_hooks(env-hooks/px4_gz_plugins.dsv)
  ament_package()
endif()
