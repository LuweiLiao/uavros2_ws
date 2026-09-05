cmake_minimum_required(VERSION 3.8)
project(bar)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/HeaderString.msg
  DEPENDENCIES std_msgs
)

add_library(bar SHARED src/hello.cpp)
target_compile_features(bar PUBLIC cxx_std_11)
target_include_directories(bar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bar Boost::thread Boost::system)

install(TARGETS bar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs rosidl_default_runtime)
ament_package()
