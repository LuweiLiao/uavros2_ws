cmake_minimum_required(VERSION 3.8)
project(glog_catkin)

# ``glog_catkin`` is retained at its original package boundary.  The ROS 1
# package was a catkin wrapper around the system Google glog installation; in
# ROS 2 the equivalent wrapper is an ament package exporting the upstream
# CMake target.  Building glog itself remains the responsibility of the
# declared system dependency (libgoogle-glog-dev), just as the normal ROS 2
# dependency workflow expects.
find_package(ament_cmake REQUIRED)
# Prefer the package's historical finder.  Some Ubuntu glog builds export a
# CMake config that unconditionally requires an optional ``Unwind`` config;
# the finder below only needs the public headers and library and therefore
# works with both distro and locally prefixed installations.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(Glog REQUIRED MODULE)

if(NOT TARGET glog::glog)
  add_library(glog::glog UNKNOWN IMPORTED GLOBAL)
  set_target_properties(glog::glog PROPERTIES
    IMPORTED_LOCATION "${GLOG_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}")
endif()

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(FILES README.md fix-unused-typedef-warning.patch
        DESTINATION share/${PROJECT_NAME})

ament_export_include_directories("${GLOG_INCLUDE_DIR}")
ament_export_libraries("${GLOG_LIBRARY}")
ament_package()
