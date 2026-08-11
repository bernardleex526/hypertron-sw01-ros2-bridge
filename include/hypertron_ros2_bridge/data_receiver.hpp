#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "hypertron_ros2_bridge/protocol_handler.hpp"

namespace rclcpp {
class Node;
}

namespace hypertron_ros2_bridge {

enum class TimestampSource {
  Receive,
  DeviceNanoseconds,
  DeviceMicroseconds,
  DeviceMilliseconds,
};
enum class QuaternionOrder { Xyzw, Wxyz };

class MappingError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct DataReceiverConfig {
  std::string imu_frame{"imu_link"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_link"};
  std::string camera_frame{"camera_link"};
  std::string imu_topic{"/imu/data"};
  std::string joint_states_topic{"/joint_states"};
  std::string odom_topic{"/odom"};
  std::string robot_state_topic{"/robot_state"};
  std::string camera_topic{"/camera/image_raw"};
  TimestampSource timestamp_source{TimestampSource::Receive};
  TimestampSource odometry_timestamp_source{TimestampSource::Receive};
  QuaternionOrder quaternion_order{QuaternionOrder::Xyzw};
  std::array<double, 3> orientation_covariance_diagonal{0.02, 0.02, 0.05};
  std::array<double, 3> angular_velocity_covariance_diagonal{0.01, 0.01,
                                                             0.02};
  std::array<double, 3> linear_acceleration_covariance_diagonal{0.10, 0.10,
                                                                0.20};
  bool camera_enabled{false};
  bool odometry_scale_verified{false};
};

struct ImuSample {
  std::uint64_t timestamp_ns{};
  std::string frame_id;
  std::array<double, 3> linear_acceleration{};
  std::array<double, 3> angular_velocity{};
  std::array<double, 4> orientation{};  // ROS xyzw
  std::array<double, 9> orientation_covariance{};
  std::array<double, 9> angular_velocity_covariance{};
  std::array<double, 9> linear_acceleration_covariance{};
};

struct OdometrySample {
  std::uint64_t timestamp_ns{};
  std::string frame_id;
  std::string child_frame_id;
  std::array<double, 3> position{};
  std::array<double, 4> orientation{};  // ROS xyzw
};

ImuSample to_imu_sample(const ImuPayload& payload,
                        std::uint64_t receive_time_ns,
                        const DataReceiverConfig& config);
OdometrySample to_odometry_sample(const OdometryPayload& payload,
                                  std::uint64_t receive_time_ns,
                                  const DataReceiverConfig& config);
bool effective_emergency_stop(bool pc_latched, bool agent_latched) noexcept;

class CameraIngestState {
 public:
  explicit CameraIngestState(bool enabled) : enabled_(enabled) {}
  bool accept(const CameraChunkPayload& payload);
  std::uint32_t disabled_drops() const noexcept {
    return disabled_drops_.load();
  }

 private:
  bool enabled_{};
  std::atomic<std::uint32_t> disabled_drops_{0};
};

struct ReceiverConnectionState {
  bool ssh_connected{};
  bool agent_connected{};
  std::uint32_t protocol_rx_drops{};
  std::uint32_t rejected_joint_commands{};
  bool emergency_stop_latched{};
  std::string last_error;
};

#ifdef HYPERTRON_WITH_ROS2
class DataReceiver {
 public:
  DataReceiver(rclcpp::Node& node, DataReceiverConfig config);
  ~DataReceiver();
  DataReceiver(const DataReceiver&) = delete;
  DataReceiver& operator=(const DataReceiver&) = delete;

  void handle(const Frame& frame, std::uint64_t receive_time_ns);
  void set_connection_state(const ReceiverConnectionState& state);
  void publish_disconnected_state();
  std::uint32_t camera_errors() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
#endif

}  // namespace hypertron_ros2_bridge
