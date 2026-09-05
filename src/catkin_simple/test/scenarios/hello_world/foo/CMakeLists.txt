cmake_minimum_required(VERSION 3.8)
project(foo)

find_package(ament_cmake REQUIRED)
find_package(bar REQUIRED)
find_package(baz REQUIRED)

add_executable(my_exec src/main.cpp)
target_compile_features(my_exec PRIVATE cxx_std_11)
target_link_libraries(my_exec bar)
target_include_directories(my_exec PRIVATE
  ${bar_INCLUDE_DIRS}
  ${baz_INCLUDE_DIRS})

install(TARGETS my_exec DESTINATION lib/${PROJECT_NAME})
ament_package()
