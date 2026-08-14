#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace hypertron_ros2_bridge {

constexpr std::uint8_t kBridgeProtocolVersion = 1U;
constexpr std::uint32_t kDefaultMaxPayload = 8U * 1024U * 1024U;

enum class MessageType : std::uint8_t {
  Hello = 0x01,
  HelloAck = 0x02,
  Ping = 0x03,
  Pong = 0x04,
  CmdVelocity = 0x11,
  CmdMode = 0x12,
  CmdEstop = 0x13,
  CmdJoint = 0x14,
  Imu = 0x21,
  Sport = 0x22,
  Odometry = 0x23,
  RobotState = 0x24,
  CameraH264 = 0x25,
  Ack = 0x7E,
  Error = 0x7F,
};

enum class BridgeError : std::uint16_t {
  Protocol = 1,
  FeatureUnavailable = 2,
  InvalidCommand = 3,
  Timeout = 4,
  SdkDisconnected = 5,
  NoControlAuthority = 6,
  EmergencyStopLatched = 7,
  RobotSystemError = 8,
  SdkCallFailed = 9,
};

enum Capability : std::uint32_t {
  CapabilityImu = 1U << 0U,
  CapabilitySport = 1U << 1U,
  CapabilityOdometry = 1U << 2U,
  CapabilityCamera = 1U << 3U,
  CapabilityJoint = 1U << 4U,
  CapabilitySystemState = 1U << 5U,
};

class ProtocolError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct Frame {
  MessageType type{};
  std::uint32_t sequence{};
  std::uint64_t monotonic_time_ns{};
  std::vector<std::uint8_t> payload;

  bool operator==(const Frame& other) const;
};

class ProtocolHandler {
 public:
  explicit ProtocolHandler(std::uint32_t max_payload = kDefaultMaxPayload);

  static std::vector<std::uint8_t> encode(const Frame& frame);
  std::vector<Frame> feed(const std::uint8_t* bytes, std::size_t size);
  std::vector<Frame> feed(const std::vector<std::uint8_t>& bytes);
  std::uint32_t dropped_frames() const noexcept;
  void reset() noexcept;

 private:
  [[noreturn]] void fail(const std::string& message);

  std::uint32_t max_payload_;
  std::uint32_t dropped_frames_{0};
  std::vector<std::uint8_t> buffer_;
};

std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size);
std::uint32_t crc32_iso_hdlc(const std::vector<std::uint8_t>& data);

struct HelloPayload {
  std::uint8_t min_version{};
  std::uint8_t max_version{};
  std::uint32_t capabilities{};
  std::uint32_t instance_nonce{};
  bool operator==(const HelloPayload& o) const;
};

struct HelloAckPayload {
  std::uint8_t selected_version{};
  std::uint32_t capabilities{};
  std::uint32_t instance_nonce{};
  std::string sdk_version;
  bool operator==(const HelloAckPayload& o) const;
};

struct VelocityPayload {
  float vx{};
  float vy{};
  float vyaw{};
  bool operator==(const VelocityPayload& o) const;
};

struct ModePayload {
  std::uint16_t mode{};
  bool operator==(const ModePayload& o) const;
};

struct EstopPayload {
  bool engage{};
  bool operator==(const EstopPayload& o) const;
};

struct AckPayload {
  std::uint32_t request_sequence{};
  std::uint16_t result_code{};
  std::string text;
  bool operator==(const AckPayload& o) const;
};

struct ErrorPayload {
  std::uint32_t request_sequence{};
  BridgeError error{BridgeError::Protocol};
  std::string text;
  bool operator==(const ErrorPayload& o) const;
};

struct ImuPayload {
  std::uint64_t device_time{};
  std::array<float, 3> acceleration{};
  std::array<float, 3> angular_velocity{};
  std::array<float, 4> quaternion{};
  bool operator==(const ImuPayload& o) const;
};

struct SportPayload {
  std::uint64_t device_time{};
  std::array<float, 4> wheel_speed{};
  std::uint16_t sport_status{};
  bool operator==(const SportPayload& o) const;
};

struct OdometryPayload {
  std::uint64_t device_time{};
  std::array<double, 3> position{};
  std::array<double, 4> orientation{};
  bool operator==(const OdometryPayload& o) const;
};

struct RobotStatePayload {
  bool sdk_linked{};
  bool control_authority{};
  bool emergency_stop{};
  bool camera_available{};
  bool odometry_scale_verified{};
  std::uint8_t system_status{};
  std::uint32_t error_code{};
  std::uint32_t warning_code{};
  std::uint16_t sport_status{};
  float battery_percentage{};
  float battery_temperature{};
  float battery_voltage{};
  std::uint16_t battery_cycle_count{};
  std::uint16_t charge_status{};
  std::array<float, 4> wheel_speed{};
  std::uint32_t last_velocity_sequence{};
  std::string last_error;
  bool operator==(const RobotStatePayload& o) const;
};

struct CameraChunkPayload {
  std::uint32_t stream_id{};
  std::uint64_t receive_time_ns{};
  std::uint32_t datagram_sequence{};
  std::vector<std::uint8_t> data;
  bool operator==(const CameraChunkPayload& o) const;
};

std::vector<std::uint8_t> encode_hello(const HelloPayload& payload);
HelloPayload decode_hello(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_hello_ack(const HelloAckPayload& payload);
HelloAckPayload decode_hello_ack(const std::vector<std::uint8_t>& bytes);
void validate_hello_ack(const HelloAckPayload& payload,
                        std::uint32_t expected_nonce,
                        std::uint32_t requested_capabilities);
std::vector<std::uint8_t> encode_velocity(const VelocityPayload& payload);
VelocityPayload decode_velocity(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_mode(const ModePayload& payload);
ModePayload decode_mode(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_estop(const EstopPayload& payload);
EstopPayload decode_estop(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_ack(const AckPayload& payload);
AckPayload decode_ack(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_error(const ErrorPayload& payload);
ErrorPayload decode_error(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_imu(const ImuPayload& payload);
ImuPayload decode_imu(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_sport(const SportPayload& payload);
SportPayload decode_sport(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_odometry(const OdometryPayload& payload);
OdometryPayload decode_odometry(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_robot_state(const RobotStatePayload& payload);
RobotStatePayload decode_robot_state(const std::vector<std::uint8_t>& bytes);
std::vector<std::uint8_t> encode_camera_chunk(
    const CameraChunkPayload& payload);
CameraChunkPayload decode_camera_chunk(const std::vector<std::uint8_t>& bytes);

enum class PackingMode { Auto, Packed, NaturalAligned };

// Legacy proprietary-protocol point type. Renamed (ProtocolLidarPoint) so it
// does not collide with the project's LiDAR stream library's `LidarPoint`
// (hypertron_ros2_bridge/lidar_stream.hpp), which the direct driver node
// includes for the UDP 6100 bypass stream.
struct ProtocolLidarPoint {
  double x{};
  double y{};
  double z{};
  std::array<std::uint32_t, 4> rgba{};
};

struct PointCloudPacket {
  std::uint64_t device_time{};
  std::uint32_t total_points{};
  std::uint32_t packet_index{};
  std::vector<ProtocolLidarPoint> points;
};

struct OdometryPacket {
  std::uint64_t device_time{};
  std::array<double, 3> position{};
  std::array<double, 4> orientation{};
};

std::optional<PointCloudPacket> parse_point_cloud_packet(
    const std::vector<std::uint8_t>& datagram, PackingMode packing,
    double coordinate_scale);
std::optional<OdometryPacket> parse_odometry_packet(
    const std::vector<std::uint8_t>& datagram, PackingMode packing,
    double position_scale, double quaternion_scale);

}  // namespace hypertron_ros2_bridge
