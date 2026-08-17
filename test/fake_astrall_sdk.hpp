#pragma once

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "hypertron_ros2_bridge/astrall_sdk.hpp"

namespace hypertron_ros2_bridge {
namespace test {

// Mutex-guarded value that can be assigned by the test thread (which keeps
// the `field = value` syntax via operator=) and read thread-safely with
// get(). Used for the programmable next_* results and *-enter hooks so a
// worker thread reading them never races with the test thread rewriting them.
template <typename T>
class Programmable {
 public:
  explicit Programmable(T value = {}) : value_(std::move(value)) {}

  // Keep the natural test syntax `next_init = Result::ok();` working.
  Programmable& operator=(T value) {
    std::lock_guard<std::mutex> lock(mutex_);
    value_ = std::move(value);
    return *this;
  }

  T get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return value_;
  }

 private:
  mutable std::mutex mutex_;
  T value_;
};

// Deterministic IAstrallSdk fake for tests. Records every SDK call with its
// arguments, returns programmable results, exposes injected telemetry
// samples, and never touches the vendor library or the network.
class FakeAstrallSdk final : public IAstrallSdk {
 public:
  struct Call {
    std::string method;
    std::uint32_t timeout_ms{};
    bool authority_sdk{};
    SubscriptionFrequency frequency{SubscriptionFrequency::Disabled};
    Velocity velocity{};
    std::uint16_t mode{};
  };

  // Programmable results; every call defaults to success. Assigned by the
  // test thread with `next_init = Result::ok();`, read thread-safely inside
  // the fake methods via .get().
  Programmable<Result> next_init{Result::ok()};
  Programmable<Result> next_heartbeat{Result::ok()};
  Programmable<Result> next_authority{Result::ok()};
  Programmable<Result> next_subscribe_imu{Result::ok()};
  Programmable<Result> next_subscribe_sport{Result::ok()};
  Programmable<Result> next_subscribe_lidar{Result::ok()};
  Programmable<Result> next_move{Result::ok()};
  Programmable<Result> next_mode{Result::ok()};

  // Test-only hook invoked inside move() on the calling thread (the runtime
  // worker) before the call is recorded; it may block. Used to simulate an
  // SDK call that is in flight when stop() runs.
  Programmable<std::function<void()>> on_move_enter;
  // Test-only hook invoked inside heartbeat() on the calling thread (the
  // runtime worker) before the call is recorded; it may block to simulate
  // an SDK heartbeat call that is in flight.
  Programmable<std::function<void()>> on_heartbeat_enter;
  // Test-only hook invoked inside snapshot() on the calling thread (the
  // runtime worker) before the snapshot-block wait is entered. Used to
  // observe that the worker is parked inside a state poll.
  Programmable<std::function<void()>> on_snapshot_enter;
  // Test-only hook invoked at the end of a successful init() on the calling
  // thread (the runtime worker), after the callbacks were installed. Used to
  // simulate a status report delivered synchronously inside the init call.
  Programmable<std::function<void()>> on_init_enter;

  // Returned verbatim by snapshot(); successful request_authority and every
  // emit_sport also update the corresponding fields.
  SdkSnapshot snapshot_value;

  Result init(const SdkCallbacks& callbacks,
              std::uint32_t timeout_ms) override {
    record({"init", timeout_ms, false, {}, {}, 0});
    const Result result = next_init.get();
    if (result.success()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        callbacks_ = callbacks;
      }
      if (auto hook = on_init_enter.get(); hook) {
        // Delivers on the calling thread (the runtime worker), so the
        // status report lands while the init call is still in progress.
        hook();
      }
    }
    return result;
  }

  void deinit() noexcept override {
    record({"deinit", 0, false, {}, {}, 0});
    clear_callbacks();
  }

  Result heartbeat(std::uint32_t timeout_ms) override {
    if (auto hook = on_heartbeat_enter.get(); hook) {
      hook();
    }
    record({"heartbeat", timeout_ms, false, {}, {}, 0});
    return next_heartbeat.get();
  }

  Result request_authority(bool sdk, std::uint32_t timeout_ms) override {
    record({"request_authority", timeout_ms, sdk, {}, {}, 0});
    const Result result = next_authority.get();
    if (result.success()) {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_value.control_authority = sdk;
    }
    return result;
  }

  Result subscribe_imu(SubscriptionFrequency frequency, ImuCallback callback,
                       std::uint32_t timeout_ms) override {
    record({"subscribe_imu", timeout_ms, false, frequency, {}, 0});
    const Result result = next_subscribe_imu.get();
    if (result.success()) {
      std::lock_guard<std::mutex> lock(mutex_);
      imu_callback_ = std::move(callback);
    }
    return result;
  }

  Result subscribe_sport(SubscriptionFrequency frequency,
                         SportCallback callback,
                         std::uint32_t timeout_ms) override {
    record({"subscribe_sport", timeout_ms, false, frequency, {}, 0});
    const Result result = next_subscribe_sport.get();
    if (result.success()) {
      std::lock_guard<std::mutex> lock(mutex_);
      sport_callback_ = std::move(callback);
    }
    return result;
  }

  Result subscribe_lidar(SubscriptionFrequency frequency,
                         std::uint32_t timeout_ms) override {
    record({"subscribe_lidar", timeout_ms, false, frequency, {}, 0});
    return next_subscribe_lidar.get();
  }

  Result move(Velocity velocity, std::uint32_t timeout_ms) override {
    if (auto hook = on_move_enter.get(); hook) {
      hook();
    }
    record({"move", timeout_ms, false, {}, velocity, 0});
    return next_move.get();
  }

  Result set_mode(std::uint16_t mode, std::uint32_t timeout_ms) override {
    record({"set_mode", timeout_ms, false, {}, {}, mode});
    return next_mode.get();
  }

  SdkSnapshot snapshot() override {
    if (auto hook = on_snapshot_enter.get(); hook) {
      hook();
    }
    std::unique_lock<std::mutex> lock(mutex_);
    snapshot_cv_.wait(lock, [this] { return !snapshot_blocked_; });
    return snapshot_value;
  }

  // Test-only injection points that drive the stored project callbacks.
  void emit_status(bool linked, bool control_authority) {
    std::function<void(bool, bool)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_value.sdk_linked = linked;
      snapshot_value.control_authority = control_authority;
      callback = callbacks_.on_status;
    }
    if (callback) {
      callback(linked, control_authority);
    }
  }

  // Test-only switch that parks every snapshot() call until cleared. Used
  // to hold the worker inside the first state poll of a new session so
  // tests can act deterministically before any snapshot of that session
  // was delivered.
  void set_snapshot_blocked(bool blocked) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_blocked_ = blocked;
    if (!blocked) {
      snapshot_cv_.notify_all();
    }
  }

  void emit_imu(const ImuSample& sample) {
    std::function<void(const ImuSample&)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      callback = imu_callback_;
    }
    if (callback) {
      callback(sample);
    }
  }

  void emit_sport(const SportSample& sample) {
    std::function<void(const SportSample&)> callback;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_value.wheel_speed = sample.wheel_speed;
      callback = sport_callback_;
    }
    if (callback) {
      callback(sample);
    }
  }

  // Thread-safe programmable sport status, consumed by the runtime's
  // snapshot poll during mode-transition confirmation.
  void set_sport_status(std::uint16_t status) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_value.sport_status = status;
  }

  // Thread-safe single-field convenience setters for the snapshot fields the
  // tests most often program (sdk_linked and control_authority).
  void set_linked(bool linked) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_value.sdk_linked = linked;
  }

  void set_authority(bool authority) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_value.control_authority = authority;
  }

  // Thread-safe mutation of any snapshot field (e.g. error_code); the poll
  // loop reads the snapshot under the same lock.
  template <typename Fn>
  void set_snapshot(Fn&& mutator) {
    std::lock_guard<std::mutex> lock(mutex_);
    mutator(snapshot_value);
  }

  // Thread-safe accessors for the recorded calls.
  std::vector<Call> calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

  std::size_t call_count(const std::string& method) const {
    const std::vector<Call> snapshot_calls = calls();
    return static_cast<std::size_t>(std::count_if(
        snapshot_calls.begin(), snapshot_calls.end(),
        [&](const Call& call) { return call.method == method; }));
  }

  bool was_called(const std::string& method) const {
    return call_count(method) > 0U;
  }

 private:
  void record(Call call) {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back(std::move(call));
  }

  void clear_callbacks() {
    std::lock_guard<std::mutex> lock(mutex_);
    callbacks_ = SdkCallbacks{};
    imu_callback_ = {};
    sport_callback_ = {};
  }

  mutable std::mutex mutex_;
  std::condition_variable snapshot_cv_;
  bool snapshot_blocked_{false};
  std::vector<Call> calls_;
  SdkCallbacks callbacks_;
  ImuCallback imu_callback_;
  SportCallback sport_callback_;
};

}  // namespace test
}  // namespace hypertron_ros2_bridge
