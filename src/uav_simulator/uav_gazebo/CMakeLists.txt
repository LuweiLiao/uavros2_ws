cmake_minimum_required(VERSION 3.8)
project(uav_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
set(PRIVATE_MODELS "${CMAKE_CURRENT_SOURCE_DIR}/../../../private/uavros2_model_prviate")
# Explicit public allowlist: a checkout without the optional submodule must
# install only uni350, even when other model links are present.
install(DIRECTORY models/uni350 DESTINATION share/${PROJECT_NAME}/models)
install(FILES worlds/uni350.world DESTINATION share/${PROJECT_NAME}/worlds)
if(EXISTS "${PRIVATE_MODELS}/models")
  install(DIRECTORY "${PRIVATE_MODELS}/models/" DESTINATION share/${PROJECT_NAME}/models)
  # Keep the same relative derived-mesh links in source and installed layouts.
  install(DIRECTORY "${PRIVATE_MODELS}/derived/" DESTINATION share/${PROJECT_NAME}/derived)
  install(DIRECTORY "${PRIVATE_MODELS}/worlds/" DESTINATION share/${PROJECT_NAME}/worlds)
else()
  message(STATUS "Private models not initialized: installing uni350 only")
endif()

ament_package()
