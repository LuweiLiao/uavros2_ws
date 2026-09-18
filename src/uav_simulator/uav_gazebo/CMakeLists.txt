cmake_minimum_required(VERSION 3.8)
project(uav_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
set(PRIVATE_MODELS "${CMAKE_CURRENT_SOURCE_DIR}/../../../private/uavros2_model_prviate")
install(DIRECTORY models DESTINATION share/${PROJECT_NAME}
  PATTERN "Scorpio" EXCLUDE
  PATTERN "tsd_model" EXCLUDE
  PATTERN "usl_quadruped2" EXCLUDE
  PATTERN "usl_quadruped2_bicopter" EXCLUDE
  PATTERN "usl_quadruped3_bicopter" EXCLUDE
  PATTERN "steel.stl" EXCLUDE
  PATTERN "insulators.stl" EXCLUDE)
if(EXISTS "${PRIVATE_MODELS}/models/Scorpio")
  install(DIRECTORY "${PRIVATE_MODELS}/models/" DESTINATION share/${PROJECT_NAME}/models)
  install(DIRECTORY "${PRIVATE_MODELS}/derived/powerline_corridor/meshes/"
    DESTINATION share/${PROJECT_NAME}/models/powerline_perching/models/powerline_corridor/meshes)
else()
  message(STATUS "Private models not initialized: building public models only")
endif()
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_package()
