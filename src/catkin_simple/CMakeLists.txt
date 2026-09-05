cmake_minimum_required(VERSION 3.8)
project(catkin_simple)

# Keep the original package boundary and helper template.  catkin_simple is a
# ROS 1 build helper, so there is no ROS 2 runtime library to port; the
# ament package exists so the original package remains discoverable at its
# original path while the ROS 2 test scenario packages use native ament
# CMake directly.
find_package(ament_cmake REQUIRED)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
