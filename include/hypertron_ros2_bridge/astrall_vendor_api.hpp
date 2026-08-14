#pragma once

#include <cstdint>
#include <functional>

namespace hypertron_ros2_bridge {

// ---------------------------------------------------------------------------
// Vendor SDK function-table seam.
//
// The production adapter (DirectAstrallSdk) drives the official ASTRALL SDK
// through this table so its mapping and lifecycle behavior are executable in
// tests with an injected fake backend. Only project-owned or opaque types
// cross this boundary; the vendor interface.h never appears here.
// ---------------------------------------------------------------------------

// Project-owned copies of the vendor's scalar status payloads. The adapter
// fills these from the SDK; tests fill them directly.
struct RawSystemStatus {
  std::uint8_t sys_status{};
  std::uint32_t error_code{};
  std::uint32_t warn_code{};
};

struct RawPowerStatus {
  float soc{};
  float temp{};
  float voltage{};
  std::uint16_t cycle_count{};
  std::uint16_t charged{};
};

// Opaque vendor data callback. `data` points at a vendor-owned buffer whose
// layout is defined by the official interface.h; the adapter validates the
// length and copies the buffer before use, so vendor struct layouts never
// appear in core headers.
using RawDataCallback = std::function<void(const void* data, std::uint16_t len)>;

// Control-authority target; numeric values match the vendor AstrallAuth
// enum (SDK=1, joystick=2).
enum class AuthorityTarget : std::uint16_t {
  Sdk = 1U,
  Joystick = 2U,
};

// Telemetry topics; numeric values match the vendor AstrallSubscribeTopicId
// enum (IMU=0x0001, sport=0x0002, lidar=0x0005). LIDAR subscriptions have
// discard semantics: they only enable the robot's UDP 6100/6101 push streams
// and deliver no payload through the callback, so the adapter binds a no-op
// discard callback.
enum class SubscriptionTopic : std::uint16_t {
  Imu = 0x0001U,
  Sport = 0x0002U,
  Lidar = 0x0005U,
};

struct AstrallVendorApi {
  // Mirrors AstrallSdkInit. The vendor may invoke either callback
  // synchronously inside this call or later from its own threads.
  std::function<std::uint16_t(RawDataCallback heartbeat_cb,
                              RawDataCallback status_cb,
                              std::uint32_t timeout_ms)> init;
  std::function<void()> deinit;
  std::function<std::uint16_t(std::uint32_t timeout_ms)> heartbeat;
  std::function<std::uint16_t(AuthorityTarget target,
                              std::uint32_t timeout_ms)> request_authority;
  // `frequency` uses the vendor enum ordering (0=close..5=250 Hz), which is
  // numerically identical to SubscriptionFrequency.
  std::function<std::uint16_t(SubscriptionTopic topic,
                              std::uint16_t frequency,
                              RawDataCallback data_cb,
                              std::uint32_t timeout_ms)> subscribe;
  std::function<std::uint16_t(std::uint16_t mode,
                              std::uint32_t timeout_ms)> set_mode;
  std::function<std::uint16_t(float vx, float vy, float vyaw,
                              std::uint32_t timeout_ms)> move;
  std::function<std::uint16_t(RawSystemStatus& out)> get_system_status;
  std::function<std::uint16_t(RawPowerStatus& out)> get_power_status;
  std::function<std::uint16_t()> get_sport_status;
};

// The real binding to the official ASTRALL SDK 1.0.7 functions. Defined in
// src/direct_astrall_sdk.cpp, the only production translation unit that
// includes the vendor interface.h.
AstrallVendorApi make_real_astrall_vendor_api();

}  // namespace hypertron_ros2_bridge
