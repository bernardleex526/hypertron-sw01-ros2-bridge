#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "hypertron_ros2_bridge/astrall_vendor_api.hpp"

namespace hypertron_ros2_bridge {

// Vendor SDK 1.0.7 result codes, redeclared as plain constants so core code
// never has to include the vendor interface.h. The numeric values are part
// of the documented SDK contract.
constexpr std::uint16_t kSdkResFailed = 0x8004U;      // 执行失败/申请失败/订阅失败
constexpr std::uint16_t kSdkResTimeout = 0x8005U;     // 超时
constexpr std::uint16_t kSdkResRunning = 0x8006U;     // 执行中
constexpr std::uint16_t kSdkResSuccess = 0x8008U;     // 执行成功
constexpr std::uint16_t kSdkResInvalidParam = 0x8010U;
constexpr std::uint16_t kSdkResNotInit = 0x8011U;
constexpr std::uint16_t kSdkResRcNoRelease = 0x8020U;
constexpr std::uint16_t kSdkResBeenObtained = 0x8021U;
constexpr std::uint16_t kSdkResWithoutAuth = 0x8022U;

// Uniform SDK call outcome: raw vendor code plus a readable message.
struct Result {
  std::uint16_t code{kSdkResFailed};
  std::string message;

  bool success() const noexcept { return code == kSdkResSuccess; }

  static Result ok(std::string message = "success") {
    return Result{kSdkResSuccess, std::move(message)};
  }
  static Result failure(std::uint16_t code, std::string message) {
    return Result{code, std::move(message)};
  }
};

// Human-readable description of a vendor result code; unknown codes embed
// their hex value.
std::string describe_sdk_result_code(std::uint16_t code);

// Subscription rates; numeric order matches the vendor AstrallSubscribeFreq
// enum (CLOSE=0 .. 250HZ=5).
enum class SubscriptionFrequency : std::uint16_t {
  Disabled = 0,
  Hz1 = 1,
  Hz25 = 2,
  Hz50 = 3,
  Hz125 = 4,
  Hz250 = 5,
};

struct Velocity {
  float vx{};
  float vy{};
  float vyaw{};
};

// Project-owned copies of vendor telemetry. No vendor type or pointer ever
// appears in these payloads.
struct ImuSample {
  std::int64_t timestamp{};
  std::array<float, 3> accelerometer{};  // m/(s*s)
  std::array<float, 3> gyroscope{};      // rad/s
  std::array<float, 4> quaternion{};
  float pitch{};  // rad
  float roll{};   // rad
  float yaw{};    // rad
  float odom_x{};
  float odom_y{};
};

struct SportSample {
  std::int64_t timestamp{};
  std::array<float, 4> wheel_speed{};
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

using ImuCallback = std::function<void(const ImuSample&)>;
using SportCallback = std::function<void(const SportSample&)>;

namespace detail {
// The adapter's internal state, defined only in
// src/direct_astrall_sdk.cpp. Namespace scope keeps it accessible to the
// internal callback machinery (free functions holding weak references)
// while remaining a private implementation detail of the adapter.
struct AstrallSdkImpl;
}  // namespace detail

#ifdef HYPERTRON_ENABLE_TEST_HOOKS
// Test-only lifecycle events (compiled only when the CMake target defines
// HYPERTRON_ENABLE_TEST_HOOKS; production builds never contain this path).
// Events are emitted from adapter-internal lifecycle points so tests can
// wait on them instead of sleeping:
//  - EnteredEnsureUninitialized: a non-callback thread entered the
//    serialized "ensure Uninitialized" loop (teardown_mutex held).
//  - ShuttingDownPublished: teardown moved the state to ShuttingDown
//    (gate closed; new init generations are rejected from here on).
//  - TeardownCompleted: teardown published Uninitialized and released the
//    gate (no project callback in flight).
enum class TestEvent {
  EnteredEnsureUninitialized,
  ShuttingDownPublished,
  TeardownCompleted,
};
#endif

struct SdkCallbacks {
  // SDK status callback with (linked, control_authority). Values are copies
  // taken by the adapter before invocation and may arrive on vendor SDK
  // threads.
  std::function<void(bool linked, bool control_authority)> on_status;
};

// The narrow, vendor-free SDK boundary used by the whole runtime. Tests
// replace it with a fake; production binds it to DirectAstrallSdk.
class IAstrallSdk {
 public:
  virtual ~IAstrallSdk() = default;

  virtual Result init(const SdkCallbacks& callbacks,
                      std::uint32_t timeout_ms) = 0;
  virtual void deinit() noexcept = 0;
  virtual Result heartbeat(std::uint32_t timeout_ms) = 0;
  // true -> SDK authority, false -> transfer back to the remote controller.
  virtual Result request_authority(bool sdk, std::uint32_t timeout_ms) = 0;
  virtual Result subscribe_imu(SubscriptionFrequency frequency,
                               ImuCallback callback,
                               std::uint32_t timeout_ms) = 0;
  virtual Result subscribe_sport(SubscriptionFrequency frequency,
                                 SportCallback callback,
                                 std::uint32_t timeout_ms) = 0;
  // Discard-semantics LIDAR subscription: enables the robot's UDP 6100
  // (point cloud) / 6101 (odometry) push streams so the node can receive
  // them on its own sockets. No payload ever flows through the callback;
  // the callback is a no-op, so there is nothing to forward. The frequency
  // (a fixed non-zero value) is decided internally by the adapter and is not
  // part of this method's contract.
  virtual Result subscribe_lidar(std::uint32_t timeout_ms) = 0;
  virtual Result move(Velocity velocity, std::uint32_t timeout_ms) = 0;
  virtual Result set_mode(std::uint16_t mode, std::uint32_t timeout_ms) = 0;
  virtual SdkSnapshot snapshot() = 0;
};

// The production adapter around the vendor x86_64 ASTRALL SDK 1.0.7. Only
// its implementation file (src/direct_astrall_sdk.cpp) may include the
// vendor interface.h. Every vendor callback copies its buffer into the
// project-owned value types above before invoking project callbacks, and no
// vendor pointer ever leaves the adapter.
//
// Lifetime contract: this adapter must NEVER be destroyed on a vendor
// callback thread. The runtime owns it (e.g. via std::unique_ptr), calls
// deinit() first, and destroys it from a non-callback (runtime) thread.
// A deinit() requested inside a project callback only sets a deferred
// flag; the runtime thread must call deinit() (or any vendor entry point
// whose tail checkpoint picks the flag up) to complete the teardown.
class DirectAstrallSdk final : public IAstrallSdk {
 public:
  // Binds the real official SDK function table.
  DirectAstrallSdk();
  // Injects a vendor function table (tests use a fake backend; the seam
  // stays free of vendor types).
  explicit DirectAstrallSdk(AstrallVendorApi api);
  ~DirectAstrallSdk() override;
  DirectAstrallSdk(const DirectAstrallSdk&) = delete;
  DirectAstrallSdk& operator=(const DirectAstrallSdk&) = delete;

  Result init(const SdkCallbacks& callbacks,
              std::uint32_t timeout_ms) override;
  void deinit() noexcept override;
  Result heartbeat(std::uint32_t timeout_ms) override;
  Result request_authority(bool sdk, std::uint32_t timeout_ms) override;
  Result subscribe_imu(SubscriptionFrequency frequency, ImuCallback callback,
                       std::uint32_t timeout_ms) override;
  Result subscribe_sport(SubscriptionFrequency frequency,
                         SportCallback callback,
                         std::uint32_t timeout_ms) override;
  Result subscribe_lidar(std::uint32_t timeout_ms) override;
  Result move(Velocity velocity, std::uint32_t timeout_ms) override;
  Result set_mode(std::uint16_t mode, std::uint32_t timeout_ms) override;
  SdkSnapshot snapshot() override;

#ifdef HYPERTRON_ENABLE_TEST_HOOKS
  // Test-only lifecycle observer; see TestEvent above. Compiled only when
  // this translation unit is built with HYPERTRON_ENABLE_TEST_HOOKS.
  void set_test_hook(std::function<void(TestEvent)> hook);
#endif

 private:
  // Shared so vendor callbacks can hold weak references that never touch a
  // destroyed adapter.
  std::shared_ptr<detail::AstrallSdkImpl> impl_;

  // Serialized "ensure Uninitialized" loop used by both the non-callback
  // deinit() path and the vendor-entry tail checkpoint: holds
  // teardown_mutex so at most one teardown publisher exists at any time.
  // Returns only while observing Uninitialized with no deferred request and
  // no in-flight project callback (failed-init drains are waited out).
  void ensure_uninitialized() noexcept;
  // Deferred-teardown checkpoint executed at the tail of every vendor
  // entry point once the thread left the vendor call stack. Unlocks
  // `lifecycle` and runs the full teardown when a project callback
  // requested deinit; returns true when the caller must short-circuit.
  bool finish_with_deferred_teardown(
      std::unique_lock<std::mutex>& lifecycle) noexcept;
};

}  // namespace hypertron_ros2_bridge
