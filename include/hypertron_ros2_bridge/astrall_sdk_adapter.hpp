#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "hypertron_ros2_bridge/protocol_handler.hpp"
#include "hypertron_ros2_bridge/robot_controller.hpp"

namespace hypertron_ros2_bridge {

constexpr std::uint16_t kAstrallSuccess = 0x8008U;
constexpr std::uint16_t kAstrallModeDamping = 0xA101U;

struct SdkCallbacks {
  std::function<void(bool linked, bool control_authority)> on_link;
  std::function<void(ImuPayload)> on_imu;
  std::function<void(SportPayload)> on_sport;
};

struct SdkSnapshot {
  bool sdk_linked{};
  bool control_authority{};
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
};

class IAstrallSdk {
 public:
  virtual ~IAstrallSdk() = default;
  virtual std::uint16_t init(const SdkCallbacks& callbacks,
                             std::uint32_t timeout_ms) = 0;
  virtual void deinit() noexcept = 0;
  virtual std::uint16_t heartbeat(std::uint32_t timeout_ms) = 0;
  virtual std::uint16_t acquire_sdk_control(std::uint32_t timeout_ms) = 0;
  virtual std::uint16_t move(float vx, float vy, float vyaw,
                             std::uint32_t timeout_ms) = 0;
  virtual std::uint16_t set_mode(std::uint16_t mode,
                                 std::uint32_t timeout_ms) = 0;
  virtual SdkSnapshot snapshot() = 0;
  virtual std::string sdk_version() const = 0;
};

// The only production class that includes and calls the vendor interface.h.
// Its implementation copies all callback-owned buffers before returning.
class AstrallSdkAdapter final : public IAstrallSdk {
 public:
  AstrallSdkAdapter();
  ~AstrallSdkAdapter() override;
  AstrallSdkAdapter(const AstrallSdkAdapter&) = delete;
  AstrallSdkAdapter& operator=(const AstrallSdkAdapter&) = delete;

  std::uint16_t init(const SdkCallbacks& callbacks,
                     std::uint32_t timeout_ms) override;
  void deinit() noexcept override;
  std::uint16_t heartbeat(std::uint32_t timeout_ms) override;
  std::uint16_t acquire_sdk_control(std::uint32_t timeout_ms) override;
  std::uint16_t move(float vx, float vy, float vyaw,
                     std::uint32_t timeout_ms) override;
  std::uint16_t set_mode(std::uint16_t mode,
                         std::uint32_t timeout_ms) override;
  SdkSnapshot snapshot() override;
  std::string sdk_version() const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class IByteStream {
 public:
  virtual ~IByteStream() = default;
  // An empty result denotes EOF or a permanent stream error.
  virtual std::vector<std::uint8_t> read_some() = 0;
  virtual bool write_all(const std::vector<std::uint8_t>& bytes) = 0;
  virtual void close() noexcept = 0;
};

struct UdpDatagram {
  std::uint64_t receive_time_ns{};
  std::vector<std::uint8_t> bytes;
};

class IUdpSource {
 public:
  virtual ~IUdpSource() = default;
  virtual std::optional<UdpDatagram> receive(
      std::chrono::milliseconds timeout) = 0;
  virtual void close() noexcept = 0;
};

struct AgentUdpSources {
  IUdpSource* camera{};
  IUdpSource* lidar{};
  IUdpSource* odometry{};
};

struct AgentConfig {
  std::uint32_t init_timeout_ms{60000};
  std::uint32_t sdk_call_timeout_ms{20};
  bool auto_acquire_control{true};
  std::chrono::milliseconds heartbeat_period{100};
  std::chrono::milliseconds motion_period{20};
  std::chrono::milliseconds state_period{500};
  std::chrono::milliseconds application_timeout{500};
  std::chrono::milliseconds command_deadman{100};
  std::size_t output_queue_capacity{256};
  std::uint32_t max_payload{kDefaultMaxPayload};
  PackingMode lidar_packing{PackingMode::Auto};
  double point_coordinate_scale{1e-3};
  double odometry_position_scale{1e-6};
  double odometry_quaternion_scale{1e-6};
  bool odometry_scale_verified{false};
  bool camera_enabled{false};
};

class AgentRuntime {
 public:
  AgentRuntime(AgentConfig config, IAstrallSdk& sdk, IByteStream& stream,
               IMonotonicClock& clock, AgentUdpSources udp = {});
  ~AgentRuntime();
  AgentRuntime(const AgentRuntime&) = delete;
  AgentRuntime& operator=(const AgentRuntime&) = delete;

  int run();
  void request_stop() noexcept;
  void step_for(std::chrono::milliseconds duration);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace hypertron_ros2_bridge
