cmake_minimum_required(VERSION 3.16)
project(hypertron_ros2_bridge VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_ROS2_BRIDGE "Build the PC-side ROS2 bridge node" ON)
option(BUILD_AGENT "Build the robot-side ASTRALL agent" OFF)
option(ENABLE_CAMERA "Enable FFmpeg H.264 decoding" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
find_package(Threads REQUIRED)

add_library(hypertron_protocol STATIC src/protocol_handler.cpp)
target_include_directories(hypertron_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(GTest REQUIRED)

  add_executable(test_thread_safe_queue test/test_thread_safe_queue.cpp)
  target_link_libraries(test_thread_safe_queue PRIVATE
    hypertron_protocol GTest::gtest_main Threads::Threads)
  add_test(NAME thread_safe_queue COMMAND test_thread_safe_queue)

  add_executable(test_protocol_handler test/test_protocol_handler.cpp)
  target_link_libraries(test_protocol_handler PRIVATE
    hypertron_protocol GTest::gtest_main Threads::Threads)
  add_test(NAME protocol_handler COMMAND test_protocol_handler)
endif()

if(BUILD_ROS2_BRIDGE)
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RobotState.msg"
    DEPENDENCIES std_msgs)

  add_executable(hypertron_bridge_node src/main.cpp)
  ament_target_dependencies(hypertron_bridge_node
    geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs)
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(hypertron_bridge_node "${cpp_typesupport_target}")

  install(TARGETS hypertron_bridge_node
    DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY include/
    DESTINATION include)

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
endif()

if(BUILD_AGENT)
  if(NOT ASTRALL_SDK_ROOT)
    message(FATAL_ERROR "BUILD_AGENT requires -DASTRALL_SDK_ROOT=<sdk-root>")
  endif()
  message(STATUS "ASTRALL agent build enabled with SDK root: ${ASTRALL_SDK_ROOT}")
endif()
