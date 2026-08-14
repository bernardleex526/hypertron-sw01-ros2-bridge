cmake_minimum_required(VERSION 3.16)
project(hypertron_ros2_bridge VERSION 0.1.0 LANGUAGES C CXX)

option(BUILD_ROS2_BRIDGE "Build the PC-side direct ASTRALL ROS2 driver node" ON)

# Vendor SDK location. Defaults to the documented commissioning-host
# installation; override with -DASTRALL_SDK_ROOT=... (or the
# ASTRALL_SDK_ROOT environment variable) elsewhere. Must provide
# include/interface.h and lib/linux/x86_64/libASTRALL_SDK.so.
set(ASTRALL_SDK_ROOT "/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++"
  CACHE PATH "ASTRALL SDK 1.0.7 C++ root directory")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

install(FILES README.md SW01_MANUAL_NOTES.md REVIEW.md LICENSE
  DESTINATION share/${PROJECT_NAME})

include(CTest)
find_package(Threads REQUIRED)

# Pure core retained for the direct driver: protocol value types,
# RobotController safety logic, and telemetry mapping.
add_library(hypertron_protocol STATIC
  src/protocol_handler.cpp
  src/robot_controller.cpp)
target_include_directories(hypertron_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Read-only host-network preflight. No vendor SDK and no ROS dependency, so
# it builds and is testable in the pure configuration as well.
add_library(hypertron_network_preflight STATIC
  src/network_preflight.cpp)
target_include_directories(hypertron_network_preflight PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Direct driver runtime: worker lifecycle, reconnect, heartbeat, mode and
# motion loops. Production-reusable static library: no vendor SDK include
# and no ROS dependency, so it builds in the pure configuration and is
# linked by the Task 5 ROS node in the ROS configuration.
add_library(hypertron_direct_runtime STATIC src/direct_driver_runtime.cpp)
target_include_directories(hypertron_direct_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hypertron_direct_runtime PUBLIC
  hypertron_protocol Threads::Threads)

# Pure SW01 LiDAR stream library: parsing and bounded frame reassembly for
# the AstrallPosCloudData (UDP 6100 point cloud) and AstrallOdometryData
# (UDP 6101 odometry) datagrams. No vendor SDK and no ROS dependency, so it
# builds and is fully testable in the pure configuration.
add_library(hypertron_lidar_stream STATIC src/lidar_stream.cpp)
target_include_directories(hypertron_lidar_stream PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hypertron_lidar_stream PUBLIC Threads::Threads)

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

  add_executable(test_network_preflight test/test_network_preflight.cpp)
  target_link_libraries(test_network_preflight PRIVATE
    hypertron_network_preflight GTest::gtest_main Threads::Threads)
  add_test(NAME network_preflight COMMAND test_network_preflight)

  # Direct driver runtime: worker lifecycle, reconnect, and telemetry loops
  # driven through the programmable fakes. Links the production runtime
  # library instead of recompiling its source. No vendor SDK and no ROS
  # dependency, so it builds and runs in the pure configuration as well.
  add_executable(test_direct_driver_runtime test/test_direct_driver_runtime.cpp)
  target_link_libraries(test_direct_driver_runtime PRIVATE
    hypertron_direct_runtime hypertron_network_preflight GTest::gtest_main
    Threads::Threads)
  add_test(NAME direct_driver_runtime COMMAND test_direct_driver_runtime)

  # Pure LiDAR stream: UDP 6100/6101 parsing, frame reassembly, and the
  # receiver driven through a programmable datagram-source fake. Links the
  # production library; no vendor SDK and no ROS dependency, so it builds
  # and runs in the pure configuration as well.
  add_executable(test_lidar_stream test/test_lidar_stream.cpp)
  target_link_libraries(test_lidar_stream PRIVATE
    hypertron_lidar_stream GTest::gtest_main Threads::Threads)
  add_test(NAME lidar_stream COMMAND test_lidar_stream)
endif()

if(BUILD_ROS2_BRIDGE)
  # The direct driver is the production runtime and requires the vendor
  # x86_64 SDK at configure time. Check before any ROS discovery so the
  # diagnostic is clear even outside a ROS environment.
  # Re-derive the SDK locations on every configure: cached find results
  # would otherwise survive a changed ASTRALL_SDK_ROOT and let a missing or
  # invalid SDK configure silently with a stale success.
  unset(ASTRALL_INCLUDE_DIR CACHE)
  unset(ASTRALL_LIBRARY CACHE)
  find_path(ASTRALL_INCLUDE_DIR interface.h
    PATHS "${ASTRALL_SDK_ROOT}/include" NO_DEFAULT_PATH)
  find_library(ASTRALL_LIBRARY NAMES ASTRALL_SDK
    PATHS "${ASTRALL_SDK_ROOT}/lib/linux/x86_64" NO_DEFAULT_PATH)
  if(NOT ASTRALL_INCLUDE_DIR OR NOT ASTRALL_LIBRARY)
    message(FATAL_ERROR
      "ASTRALL SDK x86_64 header/library not found under "
      "ASTRALL_SDK_ROOT=${ASTRALL_SDK_ROOT} (expected include/interface.h "
      "and lib/linux/x86_64/libASTRALL_SDK.so)")
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2_ros REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/RobotState.msg"
    DEPENDENCIES std_msgs)

  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(package_contract test/test_package_contract.py
      TIMEOUT 600)
  endif()

  # Direct core: the vendor SDK boundary. Only src/direct_astrall_sdk.cpp
  # includes the vendor interface.h; every consumer links against this
  # target and sees only project-owned types. The vendor include directory
  # stays PRIVATE so it never leaks into downstream targets.
  add_library(hypertron_direct_core STATIC src/direct_astrall_sdk.cpp)
  target_include_directories(hypertron_direct_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_include_directories(hypertron_direct_core PRIVATE
    "${ASTRALL_INCLUDE_DIR}")
  target_link_libraries(hypertron_direct_core
    PUBLIC hypertron_network_preflight Threads::Threads
    PRIVATE "${ASTRALL_LIBRARY}")

  if(BUILD_TESTING)
    # Links the real adapter (no SDK calls are executed: the tests only use
    # the not-initialized guards) and the programmable fake.
    add_executable(test_astrall_sdk test/test_astrall_sdk.cpp)
    target_link_libraries(test_astrall_sdk PRIVATE
      hypertron_direct_core GTest::gtest_main Threads::Threads)
    get_filename_component(ASTRALL_LIBRARY_DIR "${ASTRALL_LIBRARY}" DIRECTORY)
    set_target_properties(test_astrall_sdk PROPERTIES
      BUILD_RPATH "${ASTRALL_LIBRARY_DIR}")
    add_test(NAME astrall_sdk COMMAND test_astrall_sdk)

    # Executable adapter tests: mapping and lifecycle driven through an
    # injected fake vendor backend (no vendor call is ever executed). The
    # adapter source is compiled in this target with the test-only lifecycle
    # hook seam (HYPERTRON_ENABLE_TEST_HOOKS); the production
    # hypertron_direct_core target never defines it, so production binaries
    # contain none of the hook path.
    add_executable(test_direct_astrall_sdk
      test/test_direct_astrall_sdk.cpp
      src/direct_astrall_sdk.cpp)
    target_compile_definitions(test_direct_astrall_sdk PRIVATE
      HYPERTRON_ENABLE_TEST_HOOKS)
    target_include_directories(test_direct_astrall_sdk PRIVATE
      "${CMAKE_CURRENT_SOURCE_DIR}/include"
      "${ASTRALL_INCLUDE_DIR}")
    target_link_libraries(test_direct_astrall_sdk PRIVATE
      "${ASTRALL_LIBRARY}" GTest::gtest_main Threads::Threads)
    set_target_properties(test_direct_astrall_sdk PROPERTIES
      BUILD_RPATH "${ASTRALL_LIBRARY_DIR}")
    add_test(NAME direct_astrall_sdk COMMAND test_direct_astrall_sdk)
  endif()

  # ROS 2 driver node: bridges the direct-driver runtime onto the ROS graph
  # (/cmd_vel, /robot_mode and /joint_commands in; /imu/data, /odom and
  # /robot_state out; /emergency_stop and /control_authority services). The
  # vendor library stays a real DT_NEEDED dependency of the produced ELF
  # because the executable links it directly with --no-as-needed, so the link
  # contract stays verifiable. The vendor include directory stays PRIVATE so
  # it never leaks into the ROS node sources.
  add_executable(hypertron_driver_node
    src/driver_main.cpp
    src/driver_node.cpp)
  target_include_directories(hypertron_driver_node PRIVATE
    "${ASTRALL_INCLUDE_DIR}"
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS})
  target_link_options(hypertron_driver_node PRIVATE "-Wl,--no-as-needed")
  target_link_libraries(hypertron_driver_node PRIVATE
    hypertron_direct_core
    hypertron_direct_runtime
    hypertron_lidar_stream
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    std_srvs::std_srvs__rosidl_typesupport_cpp
    tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
    tf2_ros::tf2_ros
    ${PROJECT_NAME}__rosidl_generator_cpp
    ${PROJECT_NAME}__rosidl_typesupport_cpp
    "${ASTRALL_LIBRARY}" Threads::Threads)
  get_filename_component(ASTRALL_LIBRARY_DIR "${ASTRALL_LIBRARY}" DIRECTORY)
  set_target_properties(hypertron_driver_node PROPERTIES
    BUILD_RPATH "${ASTRALL_LIBRARY_DIR}"
    INSTALL_RPATH "${ASTRALL_LIBRARY_DIR}")

  if(BUILD_TESTING)
    # ROS graph integration tests for the driver node. Links the production
    # runtime and the (pure, vendor-free) fakes from test/; never links the
    # vendor library because no adapter code is compiled in this target.
    add_executable(test_driver_node
      test/test_driver_node.cpp
      src/driver_node.cpp)
    target_include_directories(test_driver_node PRIVATE
      test
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${geometry_msgs_INCLUDE_DIRS}
      ${nav_msgs_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS}
      ${std_msgs_INCLUDE_DIRS}
      ${std_srvs_INCLUDE_DIRS}
      ${tf2_ros_INCLUDE_DIRS})
    target_link_libraries(test_driver_node PRIVATE
      hypertron_direct_runtime hypertron_direct_core hypertron_lidar_stream
      rclcpp::rclcpp
      geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
      geometry_msgs::geometry_msgs__rosidl_generator_cpp
      nav_msgs::nav_msgs__rosidl_typesupport_cpp
      sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
      std_msgs::std_msgs__rosidl_typesupport_cpp
      std_srvs::std_srvs__rosidl_typesupport_cpp
      tf2_msgs::tf2_msgs__rosidl_typesupport_cpp
      tf2_ros::tf2_ros
      ${PROJECT_NAME}__rosidl_generator_cpp
      ${PROJECT_NAME}__rosidl_typesupport_cpp
      GTest::gtest_main Threads::Threads)
    # The generated message type-support libraries (libhypertron_ros2_bridge__*)
    # build into the same directory as the test binary; make that resolvable
    # for the loader and for RCL's type-support lookup.
    set_target_properties(test_driver_node PROPERTIES
      BUILD_RPATH "${CMAKE_BINARY_DIR}")
    add_test(NAME driver_node COMMAND test_driver_node)
    set_tests_properties(driver_node PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=$ENV{LD_LIBRARY_PATH}:${CMAKE_BINARY_DIR}")
  endif()

  install(TARGETS hypertron_driver_node hypertron_direct_core
    hypertron_network_preflight
    DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY include/
    DESTINATION include)

  ament_export_dependencies(rosidl_default_runtime rclcpp geometry_msgs
    nav_msgs sensor_msgs std_msgs std_srvs tf2_ros)
  ament_package()

  message(STATUS "ASTRALL SDK x86_64: ${ASTRALL_LIBRARY}")
endif()
