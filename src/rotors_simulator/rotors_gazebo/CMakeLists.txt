cmake_minimum_required(VERSION 3.8)
project(rotors_gazebo)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# The generated mav_msgs headers live below the rosidl build/install include
# tree, while the original public conversion headers remain at include/mav_msgs.
# Keep both include locations available without changing the public header names.
set(MAV_MSGS_INCLUDE_PREFIX "${mav_msgs_DIR}/../../../include")

add_executable(waypoint_publisher src/waypoint_publisher.cpp)
target_compile_features(waypoint_publisher PRIVATE cxx_std_17)
target_include_directories(waypoint_publisher PRIVATE
  ${MAV_MSGS_INCLUDE_PREFIX})
target_link_libraries(waypoint_publisher Eigen3::Eigen)
ament_target_dependencies(waypoint_publisher
  geometry_msgs mav_msgs nav_msgs rclcpp trajectory_msgs)

add_executable(waypoint_publisher_file src/waypoint_publisher_file.cpp)
target_compile_features(waypoint_publisher_file PRIVATE cxx_std_17)
target_include_directories(waypoint_publisher_file PRIVATE
  ${MAV_MSGS_INCLUDE_PREFIX})
target_link_libraries(waypoint_publisher_file Eigen3::Eigen)
ament_target_dependencies(waypoint_publisher_file
  geometry_msgs mav_msgs nav_msgs rclcpp sensor_msgs trajectory_msgs)

add_executable(hovering_example src/hovering_example.cpp)
target_compile_features(hovering_example PRIVATE cxx_std_17)
target_include_directories(hovering_example PRIVATE
  ${MAV_MSGS_INCLUDE_PREFIX})
target_link_libraries(hovering_example Eigen3::Eigen)
ament_target_dependencies(hovering_example
  geometry_msgs mav_msgs nav_msgs rclcpp std_srvs trajectory_msgs)

install(DIRECTORY launch models resource worlds
  DESTINATION share/${PROJECT_NAME})

install(TARGETS waypoint_publisher waypoint_publisher_file hovering_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
