cmake_minimum_required(VERSION 3.8)
project(simulator)

set(PM01_SIM_ASSETS_ROOT "" CACHE PATH "Pinned simulator assets prepared by fetch_sim_dependencies.py")
set(PM01_MESHES "${PM01_SIM_ASSETS_ROOT}/simulator/config/robot/pm01_lidar_v1/meshes")
if(NOT EXISTS "${PM01_MESHES}/LINK_BASE.dae")
  message(FATAL_ERROR "Simulator meshes missing; run scripts/build.sh sim to prepare the pinned assets")
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(pm01_sdk_pd_controller SHARED
  src/pm01_sdk_pd_controller.cpp
)
target_include_directories(pm01_sdk_pd_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pm01_sdk_pd_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  std_msgs
)
pluginlib_export_plugin_description_file(
  controller_interface pm01_sdk_pd_controller_plugin.xml
)

add_executable(livox_pointcloud_bridge
  src/livox_pointcloud_bridge.cpp
)
target_include_directories(livox_pointcloud_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(livox_pointcloud_bridge
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

add_library(pm01_stand_reset SHARED
  src/pm01_stand_reset_plugin.cpp
)
target_include_directories(pm01_stand_reset PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_directories(pm01_stand_reset PRIVATE
  ${GAZEBO_LIBRARY_DIRS}
)
ament_target_dependencies(pm01_stand_reset
  gazebo_msgs
  gazebo_ros
  rclcpp
  std_msgs
)
target_link_libraries(pm01_stand_reset
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS
    livox_pointcloud_bridge
    pm01_sdk_pd_controller
    pm01_stand_reset
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pm01_initial_pose.py
    scripts/pm01_reset_stand.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pm01_pd_math_test test/pm01_pd_math_test.cpp)
  target_include_directories(pm01_pd_math_test PRIVATE include)
  ament_add_gtest(
    livox_pointcloud_bridge_test
    test/livox_pointcloud_bridge_test.cpp
    src/livox_pointcloud_bridge.cpp
  )
  target_compile_definitions(
    livox_pointcloud_bridge_test PRIVATE SIMULATOR_LIVOX_BRIDGE_NO_MAIN
  )
  target_include_directories(livox_pointcloud_bridge_test PRIVATE include)
  ament_target_dependencies(livox_pointcloud_bridge_test
    livox_ros_driver2
    rclcpp
    sensor_msgs
  )
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  controller_interface
  hardware_interface
  livox_ros_driver2
  pluginlib
  rclcpp
  realtime_tools
  sensor_msgs
  std_msgs
)
install(DIRECTORY "${PM01_MESHES}" DESTINATION share/${PROJECT_NAME}/config/robot/pm01_lidar_v1)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../../LICENSE"
  DESTINATION share/${PROJECT_NAME})
ament_package()
