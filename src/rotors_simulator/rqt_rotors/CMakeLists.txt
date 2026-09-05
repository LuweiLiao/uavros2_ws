cmake_minimum_required(VERSION 3.8)
project(rqt_rotors)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Keep the original Python module and source directory.  Only the build system
# changes from catkin_python_setup() to the ROS 2 ament equivalent.
ament_python_install_package(rqt_rotors PACKAGE_DIR src/rqt_rotors)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/hil_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  mavros_msgs
  rclpy
)

ament_package()
