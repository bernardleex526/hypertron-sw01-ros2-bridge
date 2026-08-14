#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hypertron_ros2_bridge/astrall_sdk.hpp"
#include "hypertron_ros2_bridge/astrall_vendor_api.hpp"

namespace hypertron_ros2_bridge {
namespace test {

// Fake buffers mirror the documented layout of the official interface.h
// structs without including the vendor header:
//   AstrallImuData:  timestamp(int64) @0, accelerometer[3] @8, gyroscope[3]
//                    @20, quaternion[4] @32, pitch @48, roll @52, yaw @56,
//                    odomX @60, odomY @64 (all floats 4-aligned).
//   AstrallSportData: timestamp(int64) @0, wheelSpeed[4] @8.
inline std::vector<std::uint8_t> make_imu_buffer(
    std::int64_t timestamp, const std::array<float, 3>& accelerometer,
    const std::array<float, 3>& gyroscope, const std::array<float, 4>& quaternion,
    float pitch, float roll, float yaw, float odom_x, float odom_y) {
  std::vector<std::uint8_t> buffer(256, 0);
  std::memcpy(buffer.data() + 0, &timestamp, 8);
  std::memcpy(buffer.data() + 8, accelerometer.data(), 12);
  std::memcpy(buffer.data() + 20, gyroscope.data(), 12);
  std::memcpy(buffer.data() + 32, quaternion.data(), 16);
  std::memcpy(buffer.data() + 48, &pitch, 4);
  std::memcpy(buffer.data() + 52, &roll, 4);
  std::memcpy(buffer.data() + 56, &yaw, 4);
  std::memcpy(buffer.data() + 60, &odom_x, 4);
  std::memcpy(buffer.data() + 64, &odom_y, 4);
  return buffer;
}

inline std::vector<std::uint8_t> make_sport_buffer(
    std::int64_t timestamp, const std::array<float, 4>& wheel_speed) {
  std::vector<std::uint8_t> buffer(128, 0);
  std::memcpy(buffer.data() + 0, &timestamp, 8);
  std::memcpy(buffer.data() + 8, wheel_speed.data(), 16);
  return buffer;
}

// Programmable fake of the vendor SDK function table. State lives behind a
// shared_ptr because the adapter copies function objects into std::function.
class FakeAstrallVendorApi {
 public:
  struct Call {
    std::string method;
    std::uint32_t timeout_ms{};
    AuthorityTarget authority{AuthorityTarget::Sdk};
    SubscriptionTopic topic{SubscriptionTopic::Imu};
    std::uint16_t frequency{};
    std::uint16_t mode{};
    float vx{};
    float vy{};
    float vyaw{};
  };

  // How the fake's init delivers the status callback synchronously, if at
  // all. Valid: 1-byte buffer with bit0=link, bit1=ctrlAuthority. Empty:
  // nullptr with len 0. Short: non-null with len 0.
  enum class StatusDelivery { None, Valid, Empty, Short };

  struct State {
    std::mutex mutex;
    std::vector<Call> calls;

    RawDataCallback heartbeat_cb;
    RawDataCallback status_cb;
    RawDataCallback imu_cb;
    RawDataCallback sport_cb;

    std::uint16_t next_init{kSdkResSuccess};
    std::uint16_t next_heartbeat{kSdkResSuccess};
    std::uint16_t next_authority{kSdkResSuccess};
    std::uint16_t next_subscribe{kSdkResSuccess};
    std::uint16_t next_mode{kSdkResSuccess};
    std::uint16_t next_move{kSdkResSuccess};
    std::uint16_t next_system{kSdkResSuccess};
    std::uint16_t next_power{kSdkResSuccess};

    StatusDelivery init_status{StatusDelivery::None};
    std::uint8_t init_status_byte{0x03U};  // bit0=link, bit1=ctrlAuthority

    bool subscribe_sync_imu{false};
    std::vector<std::uint8_t> sync_imu;
    bool subscribe_sync_sport{false};
    std::vector<std::uint8_t> sync_sport;

    RawSystemStatus system;
    RawPowerStatus power;
    std::uint16_t sport_status{0xB104U};

    int deinit_calls{0};
    // Thread that executed the fake's vendor deinit (for teardown-thread
    // assertions).
    std::thread::id deinit_thread_id{};
    // Optional blocker invoked inside the fake deinit (outside the state
    // mutex) until it returns true; used to prove teardown never holds the
    // gate mutex while calling vendor deinit.
    std::function<bool()> deinit_block_until;
    bool deinit_done{false};
  };

  FakeAstrallVendorApi() : state_(std::make_shared<State>()) {}

  std::shared_ptr<State> state_;

  void record(Call call) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->calls.push_back(std::move(call));
  }

  std::vector<Call> calls() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->calls;
  }

  std::size_t call_count(const std::string& method) const {
    const std::vector<Call> snapshot = calls();
    std::size_t count = 0;
    for (const Call& call : snapshot) {
      if (call.method == method) {
        ++count;
      }
    }
    return count;
  }

  int deinit_calls() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->deinit_calls;
  }

  std::thread::id deinit_thread_id() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->deinit_thread_id;
  }

  bool deinit_done() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->deinit_done;
  }

  // Test-side emission into the stored raw data callbacks.
  void emit_status(std::uint8_t byte) {
    RawDataCallback cb;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      cb = state_->status_cb;
    }
    if (cb) {
      cb(&byte, 1);
    }
  }

  void emit_status_empty() {
    RawDataCallback cb;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      cb = state_->status_cb;
    }
    if (cb) {
      cb(nullptr, 0);
    }
  }

  void emit_status_short() {
    RawDataCallback cb;
    std::uint8_t byte = 0;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      cb = state_->status_cb;
    }
    if (cb) {
      cb(&byte, 0);
    }
  }

  void emit_imu(const std::vector<std::uint8_t>& buffer) {
    RawDataCallback cb;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      cb = state_->imu_cb;
    }
    if (cb) {
      cb(buffer.data(), static_cast<std::uint16_t>(buffer.size()));
    }
  }

  void emit_sport(const std::vector<std::uint8_t>& buffer) {
    RawDataCallback cb;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      cb = state_->sport_cb;
    }
    if (cb) {
      cb(buffer.data(), static_cast<std::uint16_t>(buffer.size()));
    }
  }

  AstrallVendorApi api() {
    const std::shared_ptr<State> s = state_;
    AstrallVendorApi a;

    a.init = [s](RawDataCallback heartbeat_cb, RawDataCallback status_cb,
                 std::uint32_t timeout_ms) -> std::uint16_t {
      record_at(s, {"init", timeout_ms, {}, {}, {}, 0, 0, 0, 0});
      RawDataCallback status = status_cb;
      {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->heartbeat_cb = std::move(heartbeat_cb);
        s->status_cb = std::move(status_cb);
      }
      // Synchronous delivery during the vendor init call, if programmed.
      switch (s->init_status) {
        case StatusDelivery::Valid:
          if (status) {
            status(&s->init_status_byte, 1);
          }
          break;
        case StatusDelivery::Empty:
          if (status) {
            status(nullptr, 0);
          }
          break;
        case StatusDelivery::Short:
          if (status) {
            status(&s->init_status_byte, 0);
          }
          break;
        case StatusDelivery::None:
          break;
      }
      return s->next_init;
    };

    a.deinit = [s]() {
      record_at(s, {"deinit", 0, {}, {}, {}, 0, 0, 0, 0});
      {
        std::lock_guard<std::mutex> lock(s->mutex);
        ++s->deinit_calls;
        s->deinit_thread_id = std::this_thread::get_id();
      }
      // Optional programmable blocker (polled outside the state mutex):
      // proves the adapter never holds its gate mutex while calling vendor
      // deinit, otherwise a concurrent delivery could deadlock. The long
      // deadline must never be reached in a correct adapter: it only
      // guarantees the fake itself cannot hang a broken test run forever.
      if (s->deinit_block_until) {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(60);
        while (std::chrono::steady_clock::now() < deadline) {
          if (s->deinit_block_until()) {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
      {
        std::lock_guard<std::mutex> lock(s->mutex);
        s->deinit_done = true;
      }
    };

    a.heartbeat = [s](std::uint32_t timeout_ms) -> std::uint16_t {
      record_at(s, {"heartbeat", timeout_ms, {}, {}, {}, 0, 0, 0, 0});
      return s->next_heartbeat;
    };

    a.request_authority = [s](AuthorityTarget target,
                              std::uint32_t timeout_ms) -> std::uint16_t {
      record_at(s, {"request_authority", timeout_ms, target, {}, {}, 0, 0, 0, 0});
      return s->next_authority;
    };

    a.subscribe = [s](SubscriptionTopic topic, std::uint16_t frequency,
                      RawDataCallback data_cb,
                      std::uint32_t timeout_ms) -> std::uint16_t {
      record_at(s, {"subscribe", timeout_ms, {}, topic, frequency, 0, 0, 0, 0});
      RawDataCallback data = data_cb;
      {
        std::lock_guard<std::mutex> lock(s->mutex);
        if (topic == SubscriptionTopic::Imu) {
          s->imu_cb = std::move(data_cb);
        } else {
          s->sport_cb = std::move(data_cb);
        }
      }
      // Synchronous first-frame delivery during the vendor subscribe call.
      if (topic == SubscriptionTopic::Imu && s->subscribe_sync_imu &&
          !s->sync_imu.empty()) {
        data(s->sync_imu.data(), static_cast<std::uint16_t>(s->sync_imu.size()));
      }
      if (topic == SubscriptionTopic::Sport && s->subscribe_sync_sport &&
          !s->sync_sport.empty()) {
        data(s->sync_sport.data(),
             static_cast<std::uint16_t>(s->sync_sport.size()));
      }
      return s->next_subscribe;
    };

    a.set_mode = [s](std::uint16_t mode,
                     std::uint32_t timeout_ms) -> std::uint16_t {
      record_at(s, {"set_mode", timeout_ms, {}, {}, {}, mode, 0, 0, 0});
      return s->next_mode;
    };

    a.move = [s](float vx, float vy, float vyaw,
                 std::uint32_t timeout_ms) -> std::uint16_t {
      record_at(s, {"move", timeout_ms, {}, {}, {}, 0, vx, vy, vyaw});
      return s->next_move;
    };

    a.get_system_status = [s](RawSystemStatus& out) -> std::uint16_t {
      record_at(s, {"get_system_status", 0, {}, {}, {}, 0, 0, 0, 0});
      out = s->system;
      return s->next_system;
    };

    a.get_power_status = [s](RawPowerStatus& out) -> std::uint16_t {
      record_at(s, {"get_power_status", 0, {}, {}, {}, 0, 0, 0, 0});
      out = s->power;
      return s->next_power;
    };

    a.get_sport_status = [s]() -> std::uint16_t {
      record_at(s, {"get_sport_status", 0, {}, {}, {}, 0, 0, 0, 0});
      return s->sport_status;
    };

    return a;
  }

 private:
  static void record_at(const std::shared_ptr<State>& s, Call call) {
    std::lock_guard<std::mutex> lock(s->mutex);
    s->calls.push_back(std::move(call));
  }
};

}  // namespace test
}  // namespace hypertron_ros2_bridge
