cmake_minimum_required(VERSION 3.8)
project(baz)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(Boost)
ament_package()
