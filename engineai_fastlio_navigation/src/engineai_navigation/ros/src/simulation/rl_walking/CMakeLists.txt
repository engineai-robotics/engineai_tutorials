cmake_minimum_required(VERSION 3.8)
project(rl_walking)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

set(PM01_SIM_ASSETS_ROOT "" CACHE PATH "Pinned simulator assets prepared by fetch_sim_dependencies.py")
set(MNN_ROOT "${PM01_SIM_ASSETS_ROOT}/rl_walking/third_party/mnn")
set(PM01_POLICY "${PM01_SIM_ASSETS_ROOT}/rl_walking/config/policy")
if(NOT EXISTS "${PM01_POLICY}/251009_221907_41000.mnn")
  message(FATAL_ERROR "Walking policy missing; run scripts/build.sh sim to prepare the pinned assets")
endif()
find_path(MNN_INCLUDE_DIR
  NAMES MNN/Interpreter.hpp
  PATHS "${MNN_ROOT}/include"
  NO_DEFAULT_PATH
)
find_library(MNN_LIBRARY
  NAMES MNN
  PATHS "${MNN_ROOT}/lib"
  NO_DEFAULT_PATH
)
if(NOT MNN_INCLUDE_DIR OR NOT MNN_LIBRARY)
  message(FATAL_ERROR
    "Pinned MNN was not found under ${MNN_ROOT}"
  )
endif()

add_executable(pm01_walking_policy_node
  src/pm01_walking_policy_node.cpp
)
target_include_directories(pm01_walking_policy_node PRIVATE
  ${MNN_INCLUDE_DIR}
)
ament_target_dependencies(pm01_walking_policy_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(pm01_walking_policy_node
  Eigen3::Eigen
  yaml-cpp
  ${MNN_LIBRARY}
)
# Executable installs to lib/${PROJECT_NAME}; libMNN.so installs to lib/.
set_target_properties(pm01_walking_policy_node PROPERTIES
  BUILD_RPATH "${MNN_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN/.."
)

install(
  TARGETS pm01_walking_policy_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES
    "${MNN_ROOT}/lib/libMNN.so"
    "${MNN_ROOT}/lib/libMNN_Express.so"
  DESTINATION lib
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/joy_velocity_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY "${PM01_POLICY}" DESTINATION share/${PROJECT_NAME}/config)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../../LICENSE"
  DESTINATION share/${PROJECT_NAME})
install(FILES "${MNN_ROOT}/LICENSE.txt"
  DESTINATION share/${PROJECT_NAME}/third_party/mnn)
ament_package()
