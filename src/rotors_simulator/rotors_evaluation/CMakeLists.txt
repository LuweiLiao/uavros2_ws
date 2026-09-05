cmake_minimum_required(VERSION 3.8)
project(rotors_evaluation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Keep the original Python package and executable names.  The only adapter is
# inside rosbag_tools/analyze_bag.py, where the ROS 1 rosbag API is replaced by
# the ROS 2 rosbag2_py reader.
ament_python_install_package(rosbag_tools PACKAGE_DIR src/rosbag_tools)

install(PROGRAMS
  src/disturbance_eval.py
  src/hovering_eval.py
  src/waypoints_eval.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
