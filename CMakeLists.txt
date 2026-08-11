cmake_minimum_required(VERSION 3.16)
project(hypertron_ros2_bridge VERSION 0.1.0 LANGUAGES CXX)

option(BUILD_ROS2_BRIDGE "Build the PC-side ROS2 bridge node" ON)
option(BUILD_AGENT "Build the robot-side ASTRALL agent" OFF)
option(ENABLE_CAMERA "Enable FFmpeg H.264 decoding" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

install(FILES README.md SW01_MANUAL_NOTES.md
  DESTINATION share/${PROJECT_NAME})

include(CTest)
find_package(Threads REQUIRED)

add_library(hypertron_protocol STATIC
  src/data_mapping.cpp
  src/protocol_handler.cpp
  src/robot_controller.cpp)
target_include_directories(hypertron_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(hypertron_agent_core STATIC
  src/agent_runtime.cpp
  src/astrall_sdk_adapter.cpp)
target_include_directories(hypertron_agent_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hypertron_agent_core PUBLIC
  hypertron_protocol Threads::Threads)

add_library(hypertron_ssh STATIC src/ssh_tunnel.cpp)
target_include_directories(hypertron_ssh PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hypertron_ssh PUBLIC
  hypertron_protocol Threads::Threads)

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

  add_executable(test_robot_controller test/test_robot_controller.cpp)
  target_link_libraries(test_robot_controller PRIVATE
    hypertron_protocol GTest::gtest_main Threads::Threads)
  add_test(NAME robot_controller COMMAND test_robot_controller)

  add_executable(test_agent_runtime test/test_agent_runtime.cpp)
  target_link_libraries(test_agent_runtime PRIVATE
    hypertron_agent_core GTest::gtest_main Threads::Threads)
  add_test(NAME agent_runtime COMMAND test_agent_runtime)

  add_executable(test_ssh_tunnel test/test_ssh_tunnel.cpp)
  target_link_libraries(test_ssh_tunnel PRIVATE
    hypertron_ssh GTest::gtest_main Threads::Threads)
  add_test(NAME ssh_tunnel COMMAND test_ssh_tunnel)

  add_executable(test_data_receiver test/test_data_receiver.cpp)
  target_link_libraries(test_data_receiver PRIVATE
    hypertron_protocol GTest::gtest_main Threads::Threads)
  add_test(NAME data_receiver COMMAND test_data_receiver)
endif()

if(BUILD_ROS2_BRIDGE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBSSH REQUIRED IMPORTED_TARGET libssh)
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  target_compile_definitions(hypertron_ssh PRIVATE HYPERTRON_WITH_LIBSSH=1)
  target_link_libraries(hypertron_ssh PRIVATE PkgConfig::LIBSSH)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RobotState.msg"
    DEPENDENCIES std_msgs)

  add_executable(hypertron_bridge_node
    src/bridge_node.cpp
    src/data_receiver.cpp
    src/main.cpp)
  target_compile_definitions(hypertron_bridge_node PRIVATE
    HYPERTRON_WITH_ROS2=1)
  ament_target_dependencies(hypertron_bridge_node
    geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs)
  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(hypertron_bridge_node
    hypertron_protocol hypertron_ssh Threads::Threads
    "${cpp_typesupport_target}")

  if(ENABLE_CAMERA)
    pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
      libavcodec libavutil libswscale)
    target_compile_definitions(hypertron_bridge_node PRIVATE
      HYPERTRON_ENABLE_CAMERA=1)
    target_link_libraries(hypertron_bridge_node PkgConfig::FFMPEG)
  endif()

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
  find_path(ASTRALL_INCLUDE_DIR interface.h
    PATHS
      "${ASTRALL_SDK_ROOT}/include"
      "${ASTRALL_SDK_ROOT}/C++/include"
    NO_DEFAULT_PATH REQUIRED)
  find_library(ASTRALL_LIBRARY
    NAMES ASTRALL_SDK
    PATHS
      "${ASTRALL_SDK_ROOT}/lib/linux/${CMAKE_SYSTEM_PROCESSOR}"
      "${ASTRALL_SDK_ROOT}/lib/linux/ARM64"
      "${ASTRALL_SDK_ROOT}/lib/linux/x86_64"
      "${ASTRALL_SDK_ROOT}/C++/lib/linux/${CMAKE_SYSTEM_PROCESSOR}"
      "${ASTRALL_SDK_ROOT}/C++/lib/linux/ARM64"
      "${ASTRALL_SDK_ROOT}/C++/lib/linux/x86_64"
    NO_DEFAULT_PATH REQUIRED)
  target_compile_definitions(hypertron_agent_core PRIVATE
    HYPERTRON_WITH_ASTRALL_SDK=1)
  target_include_directories(hypertron_agent_core PRIVATE
    "${ASTRALL_INCLUDE_DIR}")
  target_link_libraries(hypertron_agent_core PUBLIC "${ASTRALL_LIBRARY}")

  add_executable(hypertron_bridge_agent src/agent_main.cpp)
  target_link_libraries(hypertron_bridge_agent PRIVATE
    hypertron_agent_core Threads::Threads)
  install(TARGETS hypertron_bridge_agent DESTINATION bin)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
  message(STATUS "ASTRALL agent SDK: ${ASTRALL_LIBRARY}")
endif()
