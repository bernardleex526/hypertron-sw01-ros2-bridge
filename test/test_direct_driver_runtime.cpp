#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "fake_astrall_sdk.hpp"
#include "fake_network_preflight.hpp"
#include "hypertron_ros2_bridge/direct_driver_runtime.hpp"
#include "hypertron_ros2_bridge/network_preflight.hpp"

namespace hypertron_ros2_bridge {
namespace {

using namespace std::chrono_literals;

using test::FakeAstrallSdk;
using test::FakeNetworkPreflight;

// Bounded polling helper for fake call-history conditions. No test ever
// sleeps to sequence or assert runtime timing; every assertion waits on an
// observer event or on a call-history condition with a timeout.
bool wait_until(const std::function<bool()>& condition,
                std::chrono::milliseconds timeout = 2s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(2ms);
  }
  return condition();
}

// Real-time clock with a test-controlled offset. The runtime's waits use
// steady-clock deadlines, so the base must stay real time for the worker to
// progress; the offset drives the controller's command deadman
// deterministically (a pure manual clock would livelock the worker loop).
class OffsetClock final : public IMonotonicClock {
 public:
  time_point now() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::chrono::steady_clock::now() + offset_;
  }
  void advance(std::chrono::milliseconds duration) {
    std::lock_guard<std::mutex> lock(mutex_);
    offset_ += duration;
  }

 private:
  mutable std::mutex mutex_;
  std::chrono::steady_clock::duration offset_{};
};

// Small periods and backoff so every phase of the worker loop completes
// quickly; assertions never depend on these durations.
RuntimeConfig fast_config() {
  RuntimeConfig config;
  config.interface = "eno1";
  config.host_address = "10.18.0.200/24";
  config.robot_address = "10.18.0.100";
  config.init_timeout_ms = 1000U;
  config.sdk_call_timeout_ms = 500U;
  config.heartbeat_period = 5ms;
  config.state_poll_period = 5ms;
  config.mode_timeout_ms = 100ms;
  config.reconnect_initial_delay_ms = 10ms;
  config.reconnect_max_delay_ms = 20ms;
  config.imu_freq = SubscriptionFrequency::Hz50;
  config.sport_freq = SubscriptionFrequency::Hz25;
  return config;
}

// Preflight fake that fails a fixed number of checks before succeeding.
// Standalone (FakeNetworkPreflight is final): same shape, plus a failure
// counter so tests can exercise the bounded retry loop.
class FlakyNetworkPreflight final : public INetworkPreflight {
 public:
  int failures_remaining{0};
  NetworkDecision next_decision;
  std::vector<NetworkExpectation> expected_seen;

  NetworkDecision check(const NetworkExpectation& expected) override {
    expected_seen.push_back(expected);
    if (failures_remaining > 0) {
      --failures_remaining;
      return NetworkDecision{false, "test-injected preflight failure"};
    }
    return next_decision;
  }
};

struct RecordedStatus {
  bool linked{};
  bool authority{};
};

struct RecordedEvent {
  int severity{};
  std::string message;
};

// Thread-safe observer recording every callback; cv-notified so tests wait
// on events instead of sleeping. Not final: tests extend it to add
// stop()-in-callback behavior while keeping the recording.
class RecordingObserver : public RuntimeObserver {
 public:
  void on_state(const SdkSnapshot& snapshot) override {
    push([&] { states_.push_back(snapshot); });
  }
  void on_imu(const ImuSample& sample) override {
    push([&] { imus_.push_back(sample); });
  }
  void on_sport(const SportSample& sample) override {
    push([&] { sports_.push_back(sample); });
  }
  void on_status(bool linked, bool control_authority) override {
    push([&] { statuses_.push_back({linked, control_authority}); });
  }
  void on_event(int severity, std::string message) override {
    push([&] { events_.push_back({severity, std::move(message)}); });
  }

  bool wait_for_event(int severity, const std::string& needle,
                      std::size_t occurrence = 1,
                      std::chrono::milliseconds timeout = 2s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      std::size_t matches = 0;
      for (const RecordedEvent& event : events_) {
        if (event.severity == severity &&
            event.message.find(needle) != std::string::npos &&
            ++matches >= occurrence) {
          return true;
        }
      }
      return false;
    });
  }

  bool wait_for_status(bool linked, bool authority,
                       std::chrono::milliseconds timeout = 2s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      for (const RecordedStatus& status : statuses_) {
        if (status.linked == linked && status.authority == authority) {
          return true;
        }
      }
      return false;
    });
  }

  // Waits until a state poll observed a live link with control authority,
  // which is what the RobotController safety gate consumes.
  bool wait_for_live_state(std::chrono::milliseconds timeout = 2s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      for (const SdkSnapshot& state : states_) {
        if (state.sdk_linked && state.control_authority) {
          return true;
        }
      }
      return false;
    });
  }

  bool wait_for_imu(std::chrono::milliseconds timeout = 2s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] { return !imus_.empty(); });
  }

  bool wait_for_sport(std::chrono::milliseconds timeout = 2s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] { return !sports_.empty(); });
  }

  // Waits until a state poll delivered a snapshot matching `pred`.
  template <typename Pred>
  bool wait_for_state_matching(Pred&& pred,
                               std::chrono::milliseconds timeout = 2s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      for (const SdkSnapshot& state : states_) {
        if (pred(state)) {
          return true;
        }
      }
      return false;
    });
  }

  // Like wait_for_state_matching, but only considers polls recorded after
  // `from_count` (a state_count() snapshot taken before a mutation), so a
  // stale matching snapshot cannot satisfy the wait.
  template <typename Pred>
  bool wait_for_new_state_matching(Pred&& pred, std::size_t from_count,
                                   std::chrono::milliseconds timeout = 2s) const {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, timeout, [&] {
      for (std::size_t i = from_count; i < states_.size(); ++i) {
        if (pred(states_[i])) {
          return true;
        }
      }
      return false;
    });
  }

  // Non-blocking scans of the recorded events (events triggered by the
  // calling thread are recorded before the triggering call returns).
  bool has_event(int severity, const std::string& needle) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const RecordedEvent& event : events_) {
      if (event.severity == severity &&
          event.message.find(needle) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  std::size_t event_count(int severity, const std::string& needle) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t count = 0;
    for (const RecordedEvent& event : events_) {
      if (event.severity == severity &&
          event.message.find(needle) != std::string::npos) {
        ++count;
      }
    }
    return count;
  }

  // Index of the first matching event in the recorded order, or
  // events_.size() when absent. Used for connected-then-disconnected
  // event-order assertions.
  std::size_t first_event_index(int severity,
                                const std::string& needle) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < events_.size(); ++i) {
      if (events_[i].severity == severity &&
          events_[i].message.find(needle) != std::string::npos) {
        return i;
      }
    }
    return events_.size();
  }

  std::size_t state_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return states_.size();
  }

 private:
  template <typename Fn>
  void push(Fn&& fn) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      fn();
    }
    cv_.notify_all();
  }

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  std::vector<SdkSnapshot> states_;
  std::vector<ImuSample> imus_;
  std::vector<SportSample> sports_;
  std::vector<RecordedStatus> statuses_;
  std::vector<RecordedEvent> events_;
};

void expect_never_actuated(const FakeAstrallSdk* sdk) {
  EXPECT_FALSE(sdk->was_called("request_authority"));
  EXPECT_FALSE(sdk->was_called("set_mode"));
  EXPECT_FALSE(sdk->was_called("move"));
}

// Observer that records like RecordingObserver and additionally calls
// stop() from inside the first on_status callback that reports a lost
// control authority, simulating a node shutting down from its own status
// callback (dispatched on the fake's "vendor" thread).
class ThrowingObserver final : public RecordingObserver {
 public:
  enum Flags : unsigned {
    kNone = 0u,
    kOnState = 1u << 0,
    kOnImu = 1u << 1,
    kOnSport = 1u << 2,
    kOnStatus = 1u << 3,
    kOnEvent = 1u << 4,
  };

  explicit ThrowingObserver(unsigned flags) : flags_(flags) {}

  void on_state(const SdkSnapshot& snapshot) override {
    if (flags_ & kOnState) {
      Throw("on_state");
    }
    RecordingObserver::on_state(snapshot);
  }
  void on_imu(const ImuSample& sample) override {
    if (flags_ & kOnImu) {
      Throw("on_imu");
    }
    RecordingObserver::on_imu(sample);
  }
  void on_sport(const SportSample& sample) override {
    if (flags_ & kOnSport) {
      Throw("on_sport");
    }
    RecordingObserver::on_sport(sample);
  }
  void on_status(bool linked, bool control_authority) override {
    if (flags_ & kOnStatus) {
      Throw("on_status");
    }
    RecordingObserver::on_status(linked, control_authority);
  }
  void on_event(int severity, std::string message) override {
    if (flags_ & kOnEvent) {
      Throw("on_event");
    }
    RecordingObserver::on_event(severity, std::move(message));
  }

 private:
  [[noreturn]] static void Throw(const char* which) {
    throw std::runtime_error(std::string("throwing observer: ") + which);
  }
  unsigned flags_;
};

TEST(DirectDriverRuntime, StartupIsNonActuatingAndPollsTelemetry) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  // Raw pointers stay valid while the runtime owns the fakes; the local
  // unique_ptrs are moved into the runtime and must not be dereferenced.
  FakeAstrallSdk* sdk_ptr = sdk.get();
  FakeNetworkPreflight* preflight_ptr = preflight.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));
  ASSERT_TRUE(wait_until([&] { return observer.state_count() >= 1; }));

  // The preflight expectation carries the configured network facts.
  ASSERT_EQ(preflight_ptr->expected_seen.size(), 1U);
  EXPECT_EQ(preflight_ptr->expected_seen[0].interface, "eno1");
  EXPECT_EQ(preflight_ptr->expected_seen[0].host_address, "10.18.0.200/24");
  EXPECT_EQ(preflight_ptr->expected_seen[0].robot_address, "10.18.0.100");

  // Telemetry samples reach the observer and the latest-sample storage.
  ImuSample imu;
  imu.timestamp = 7;
  imu.yaw = 0.5F;
  sdk_ptr->emit_imu(imu);
  SportSample sport;
  sport.timestamp = 8;
  sport.wheel_speed = {1.0F, 2.0F, 3.0F, 4.0F};
  sdk_ptr->emit_sport(sport);
  ASSERT_TRUE(observer.wait_for_imu());
  ASSERT_TRUE(observer.wait_for_sport());
  EXPECT_EQ(runtime.latest_imu().timestamp, 7);
  EXPECT_EQ(runtime.latest_sport().timestamp, 8);

  // The SDK status callback is forwarded to the observer.
  sdk_ptr->emit_status(true, false);
  ASSERT_TRUE(observer.wait_for_status(true, false));

  // Startup history: lifecycle and telemetry calls only, at the configured
  // rates, with no authority, mode, or motion command.
  const std::vector<FakeAstrallSdk::Call> calls = sdk_ptr->calls();
  ASSERT_TRUE(sdk_ptr->was_called("init"));
  ASSERT_TRUE(sdk_ptr->was_called("subscribe_imu"));
  ASSERT_TRUE(sdk_ptr->was_called("subscribe_sport"));
  ASSERT_TRUE(sdk_ptr->was_called("heartbeat"));
  bool saw_imu_freq = false;
  bool saw_sport_freq = false;
  for (const FakeAstrallSdk::Call& call : calls) {
    if (call.method == "subscribe_imu") {
      saw_imu_freq = call.frequency == SubscriptionFrequency::Hz50;
    } else if (call.method == "subscribe_sport") {
      saw_sport_freq = call.frequency == SubscriptionFrequency::Hz25;
    }
  }
  EXPECT_TRUE(saw_imu_freq);
  EXPECT_TRUE(saw_sport_freq);
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, RetriesAfterPreflightFailureUntilConnected) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  auto preflight = std::make_unique<FlakyNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  preflight->failures_remaining = 1;
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  FlakyNetworkPreflight* preflight_ptr = preflight.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // The injected failure is reported, then the bounded backoff retries and
  // the runtime connects without being told to exit.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "preflight"));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // Both checks ran against the configured expectation.
  ASSERT_GE(preflight_ptr->expected_seen.size(), 2U);
  EXPECT_EQ(preflight_ptr->expected_seen.back().interface, "eno1");
  EXPECT_EQ(preflight_ptr->expected_seen.back().host_address, "10.18.0.200/24");
  EXPECT_EQ(preflight_ptr->expected_seen.back().robot_address, "10.18.0.100");

  // The retry path never actuates.
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, RetriesAfterInitFailure) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->next_init = Result::failure(kSdkResFailed, "test-injected init failure");
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // First attempt fails, is torn down, and the backoff retries.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "init"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));

  // Allow the retry to succeed; the runtime must reach the connected loop.
  sdk_ptr->next_init = Result::ok();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("init") >= 2; }));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // The failed attempt was cleaned up and nothing was actuated.
  EXPECT_GE(sdk_ptr->call_count("deinit"), 1U);
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, RetriesAfterSubscriptionFailure) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->next_subscribe_imu =
      Result::failure(kSdkResFailed, "test-injected subscription failure");
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // The injected subscription failure is reported, the broken session is
  // invalidated and deinitialized, and the bounded backoff retries.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "subscription"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));

  // The retry subscribes again and reaches the connected loop.
  sdk_ptr->next_subscribe_imu = Result::ok();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("init") >= 2; }));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // The failed attempt was cleaned up and nothing was actuated.
  EXPECT_GE(sdk_ptr->call_count("deinit"), 1U);
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, RetriesAfterSportSubscriptionFailure) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->next_subscribe_sport = Result::failure(
      kSdkResFailed, "test-injected sport subscription failure");
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // The IMU subscription succeeds, then the sport subscription fails: the
  // session is torn down and retried from preflight.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "subscription"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));

  // The retry subscribes both streams again and reaches the connected loop.
  sdk_ptr->next_subscribe_sport = Result::ok();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(
      wait_until([&] { return sdk_ptr->call_count("subscribe_sport") >= 2; }));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, LidarSubscriptionFailureRetriesConnection) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->next_subscribe_lidar = Result::failure(
      kSdkResFailed, "test-injected lidar subscription failure");
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // The IMU and sport subscriptions succeed but the LIDAR subscription fails:
  // the session is torn down and retried from preflight (same path as imu /
  // sport subscription failure).
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "subscription"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));

  // Allow the retry to succeed; the runtime subscribes lidar again and
  // reaches the connected loop. A fresh session is established (init >= 2).
  sdk_ptr->next_subscribe_lidar = Result::ok();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("init") >= 2; }));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("subscribe_lidar") >= 2; }));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // The failed attempt was cleaned up and nothing was actuated.
  EXPECT_GE(sdk_ptr->call_count("deinit"), 1U);
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, LidarStreamDisabledNeverSubscribes) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  config.enable_lidar_stream = false;

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // A connected session is established but subscribe_lidar is never called.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));
  EXPECT_EQ(sdk_ptr->call_count("subscribe_lidar"), 0U);
  // The other subscriptions still run once per session.
  EXPECT_GE(sdk_ptr->call_count("subscribe_imu"), 1U);
  EXPECT_GE(sdk_ptr->call_count("subscribe_sport"), 1U);
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, ReconnectsAfterLinkLossWithoutRestoringMotion) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // The SDK reports a link drop through the status callback.
  sdk_ptr->emit_status(false, false);
  ASSERT_TRUE(observer.wait_for_status(false, false));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));

  // The broken session is invalidated and deinitialized...
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));
  // ...and a fresh session is re-established from preflight.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("init") >= 2; }));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected", 2));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 2; }));

  // Reconnection never restores authority, mode, or velocity: the whole
  // history contains no such command.
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, SingleHeartbeatFailureDisconnectsAndReconnects) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  config.heartbeat_call_timeout_ms = 250U;
  // heartbeat_max_failures keeps its default of 1: a single failed
  // heartbeat ends the session (design: "heartbeat failure ... clears
  // pending motion and mode work").

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // The configured heartbeat call timeout is passed to the SDK.
  for (const FakeAstrallSdk::Call& call : sdk_ptr->calls()) {
    if (call.method == "heartbeat") {
      EXPECT_EQ(call.timeout_ms, 250U);
      break;
    }
  }

  // Park the failing heartbeat call inside the fake: while it is in flight
  // nothing has been recorded or settled yet, which makes the "exactly one
  // failing call precedes the disconnect" assertion deterministic instead
  // of racing the worker's next heartbeat period.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool failing_call_entered = false;
  bool release_failing_call = false;
  std::atomic<bool> first_call_after_injection{true};
  sdk_ptr->on_heartbeat_enter = [&] {
    if (!first_call_after_injection.exchange(false)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      failing_call_entered = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_failing_call; });
  };
  const std::size_t calls_before = sdk_ptr->call_count("heartbeat");
  sdk_ptr->next_heartbeat =
      Result::failure(kSdkResTimeout, "test-injected heartbeat timeout");
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(
        hook_cv.wait_for(lock, 2s, [&] { return failing_call_entered; }));
  }
  // The failing call is in flight: the fake has not recorded it yet and no
  // disconnect event exists.
  EXPECT_EQ(sdk_ptr->call_count("heartbeat"), calls_before);
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_failing_call = true;
  }
  hook_cv.notify_all();

  // The single failed heartbeat call ends the session immediately: the
  // recorded history gains exactly that one call before the disconnect
  // (the next heartbeat belongs to the reconnected session, at least one
  // backoff away).
  ASSERT_TRUE(wait_until([&] {
    return sdk_ptr->call_count("heartbeat") == calls_before + 1U;
  }));
  EXPECT_EQ(sdk_ptr->call_count("heartbeat"), calls_before + 1U);
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "heartbeat"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));

  // Event order: connected first, then the heartbeat-failure disconnect.
  const std::size_t connected_index =
      observer.first_event_index(kRuntimeEventInfo, "connected");
  const std::size_t heartbeat_failure_index =
      observer.first_event_index(kRuntimeEventError, "heartbeat");
  EXPECT_LT(connected_index, heartbeat_failure_index);

  // ...and the runtime reconnects and heartbeats again from a fresh
  // session; the whole history contains no actuating call.
  sdk_ptr->next_heartbeat = Result::ok();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected", 2));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 2; }));
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, HeartbeatFailureThresholdIsConfigurable) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  config.heartbeat_max_failures = 2U;

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // Park only the second heartbeat call after the injection: the first
  // failed call must be tolerated (threshold 2), which is observable while
  // the second one is in flight.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool second_call_entered = false;
  bool release_second_call = false;
  std::atomic<int> calls_since_injection{0};
  sdk_ptr->on_heartbeat_enter = [&] {
    if (calls_since_injection.fetch_add(1) != 1) {
      return;  // park only the second call after installation
    }
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      second_call_entered = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_second_call; });
  };
  sdk_ptr->next_heartbeat =
      Result::failure(kSdkResTimeout, "test-injected heartbeat timeout");
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return second_call_entered; }));
  }
  // The first failure was recorded and tolerated: no disconnect event yet.
  EXPECT_FALSE(observer.has_event(kRuntimeEventError, "heartbeat"));
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_second_call = true;
  }
  hook_cv.notify_all();

  // The second consecutive failure reaches the threshold: the session ends
  // and the runtime reconnects.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "heartbeat"));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));
  sdk_ptr->next_heartbeat = Result::ok();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected", 2));
  expect_never_actuated(sdk_ptr);
  runtime.stop();
}

TEST(DirectDriverRuntime, HeartbeatFailureDetectionIsBoundedByCallTimeout) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  config.heartbeat_call_timeout_ms = 200U;

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));

  // The failing heartbeat call behaves like a real SDK call that is slow:
  // it blocks the worker for 150ms (within the configured 200ms call
  // timeout) before returning the failure. The delay simulates the vendor
  // call inside the fake; the test itself only waits on events and never
  // sleeps to assert timing.
  sdk_ptr->on_heartbeat_enter = [] { std::this_thread::sleep_for(150ms); };
  sdk_ptr->next_heartbeat =
      Result::failure(kSdkResTimeout, "test-injected heartbeat timeout");
  const auto injected = std::chrono::steady_clock::now();

  // Detection latency is bounded by the heartbeat call timeout: the session
  // ends shortly after the failed call returns (about 150ms here, below
  // the 200ms timeout), not after extra counting or delay. The lower bound
  // proves the teardown waited for the in-flight call instead of racing it;
  // the upper bound would catch a scheme that needed several heartbeat
  // periods or slept the full timeout before settling.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "heartbeat"));
  const auto elapsed = std::chrono::steady_clock::now() - injected;
  EXPECT_GE(elapsed, 150ms);
  EXPECT_LT(elapsed, 300ms);
  runtime.stop();
}

TEST(DirectDriverRuntime, StopWakesBackoffImmediately) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FlakyNetworkPreflight>();
  preflight->next_decision = NetworkDecision{false, "test-injected failure"};
  preflight->failures_remaining = 100000;
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  config.reconnect_initial_delay_ms = 30s;
  config.reconnect_max_delay_ms = 30s;

  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // The worker is parked in a 30s backoff wait...
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "preflight"));

  // ...and stop() must wake it immediately instead of waiting it out.
  const auto started = std::chrono::steady_clock::now();
  runtime.stop();
  EXPECT_LT(std::chrono::steady_clock::now() - started, 1s);
}

// Drives a runtime to a live session: connected event first, then a status
// report confirming the SDK link and control authority, then a state poll
// that feeds the RobotController safety gate. No test sleeps.
bool connect_live([[maybe_unused]] DirectDriverRuntime& runtime,
                  RecordingObserver& observer, FakeAstrallSdk* sdk) {
  if (!observer.wait_for_event(kRuntimeEventInfo, "connected")) {
    return false;
  }
  sdk->emit_status(true, true);
  if (!observer.wait_for_status(true, true)) {
    return false;
  }
  return observer.wait_for_live_state();
}

// True when the fake recorded a set_mode call for exactly `mode`.
bool recorded_set_mode(const FakeAstrallSdk* sdk, std::uint16_t mode) {
  for (const FakeAstrallSdk::Call& call : sdk->calls()) {
    if (call.method == "set_mode" && call.mode == mode) {
      return true;
    }
  }
  return false;
}

bool same_velocity(const Velocity& a, const Velocity& b) {
  return a.vx == b.vx && a.vy == b.vy && a.vyaw == b.vyaw;
}

// True when the fake recorded a move call with exactly `velocity`.
bool recorded_move(const FakeAstrallSdk* sdk, const Velocity& velocity) {
  for (const FakeAstrallSdk::Call& call : sdk->calls()) {
    if (call.method == "move" && same_velocity(call.velocity, velocity)) {
      return true;
    }
  }
  return false;
}

// Index of the first recorded call matching `pred`, or calls().size() when
// no call matches. Used for relative-order assertions.
std::size_t first_call_index(
    const FakeAstrallSdk* sdk,
    const std::function<bool(const FakeAstrallSdk::Call&)>& pred) {
  const std::vector<FakeAstrallSdk::Call> calls = sdk->calls();
  for (std::size_t i = 0; i < calls.size(); ++i) {
    if (pred(calls[i])) {
      return i;
    }
  }
  return calls.size();
}

TEST(DirectDriverRuntime, AuthorityRequestsAreForwardedAndResolved) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // SDK authority request is forwarded with sdk=true and resolves with the
  // mapped success result.
  std::future<Result> first = runtime.request_authority(true);
  ASSERT_EQ(first.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(first.get().success());

  // A false request transfers authority back to the remote controller
  // (vendor joystick target) and is recorded with sdk=false.
  std::future<Result> second = runtime.request_authority(false);
  ASSERT_EQ(second.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(second.get().success());

  bool saw_sdk_request = false;
  bool saw_joystick_request = false;
  for (const FakeAstrallSdk::Call& call : sdk_ptr->calls()) {
    if (call.method != "request_authority") {
      continue;
    }
    if (call.authority_sdk) {
      saw_sdk_request = true;
    } else {
      saw_joystick_request = true;
    }
  }
  EXPECT_TRUE(saw_sdk_request);
  EXPECT_TRUE(saw_joystick_request);
  runtime.stop();
}

TEST(DirectDriverRuntime, AuthorityFailureMapsToResult) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  sdk_ptr->next_authority =
      Result::failure(kSdkResWithoutAuth, "test-injected authority failure");
  std::future<Result> future = runtime.request_authority(true);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  const Result result = future.get();
  EXPECT_EQ(result.code, kSdkResWithoutAuth);
  EXPECT_NE(result.message.find("test-injected"), std::string::npos);
  runtime.stop();
}

TEST(DirectDriverRuntime, CommandsFailImmediatelyWhileDisconnected) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FlakyNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  FlakyNetworkPreflight* preflight_ptr = preflight.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Break the session and make the reconnect attempt fail so the worker is
  // parked outside any connected session.
  preflight_ptr->failures_remaining = 100000;
  sdk_ptr->emit_status(false, false);
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "preflight"));

  // Both command kinds fail immediately (already-resolved futures) with a
  // disconnected result and are never forwarded to the SDK.
  std::future<Result> authority = runtime.request_authority(true);
  ASSERT_EQ(authority.wait_for(0ms), std::future_status::ready);
  const Result authority_result = authority.get();
  EXPECT_EQ(authority_result.code, kSdkResNotInit);
  EXPECT_NE(authority_result.message.find("disconnected"),
            std::string::npos);

  std::future<Result> mode = runtime.request_mode("stand");
  ASSERT_EQ(mode.wait_for(0ms), std::future_status::ready);
  const Result mode_result = mode.get();
  EXPECT_EQ(mode_result.code, kSdkResNotInit);
  EXPECT_NE(mode_result.message.find("disconnected"), std::string::npos);

  EXPECT_EQ(sdk_ptr->call_count("request_authority"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);
  runtime.stop();
}

TEST(DirectDriverRuntime, CommandsFailImmediatelyBeforeStart) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);

  std::future<Result> authority = runtime.request_authority(true);
  ASSERT_EQ(authority.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(authority.get().code, kSdkResNotInit);

  std::future<Result> mode = runtime.request_mode("stand");
  ASSERT_EQ(mode.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(mode.get().code, kSdkResNotInit);

  EXPECT_EQ(sdk_ptr->call_count("request_authority"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);
  runtime.stop();
}

TEST(DirectDriverRuntime, ModeRequestsAreSerializedAndConfirmed) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_sport_status(0xB101U);  // damping: not the target
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // First request is accepted and stays pending until the stable sport
  // status (stand -> B102) is observed.
  std::future<Result> stand = runtime.request_mode("stand");
  EXPECT_NE(stand.wait_for(0ms), std::future_status::ready);

  // While the transition is pending a second mode request is rejected
  // immediately with the executing code.
  std::future<Result> rejected = runtime.request_mode("down");
  ASSERT_EQ(rejected.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(rejected.get().code, kSdkResRunning);

  // The worker executed set_mode with the stand mode code before the
  // confirmation poll saw the stable status.
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA102U); }));
  sdk_ptr->set_sport_status(0xB102U);
  ASSERT_EQ(stand.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(stand.get().success());

  // The completed transition released the serialization: a new request is
  // accepted, executed, and confirmed with its own stable status.
  std::future<Result> down = runtime.request_mode("down");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA103U); }));
  sdk_ptr->set_sport_status(0xB103U);
  ASSERT_EQ(down.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(down.get().success());
  runtime.stop();
}

TEST(DirectDriverRuntime, AuthorityLossAbortsModeTransitionAndNextModeIsSafe) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB101U);  // damping: not the requested target
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Start a transition that will not reach its stable status.
  std::future<Result> stand = runtime.request_mode("stand");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA102U); }));

  // Authority loss clears the controller-side transition. The worker must
  // settle the orphaned worker-side transition without leaving its promise
  // dangling forever.
  sdk_ptr->emit_status(true, false);
  ASSERT_EQ(stand.wait_for(2s), std::future_status::ready);
  const Result stand_result = stand.get();
  EXPECT_FALSE(stand_result.success());
  EXPECT_NE(stand_result.message.find("safety gate"), std::string::npos);

  // Regain authority and issue another mode command. The previous runtime
  // transition is already cleared, so the new promise must not overwrite an
  // unresolved old one.
  sdk_ptr->emit_status(true, true);
  ASSERT_TRUE(observer.wait_for_status(true, true));
  std::future<Result> down = runtime.request_mode("down");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA103U); }));
  sdk_ptr->set_sport_status(0xB103U);
  ASSERT_EQ(down.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(down.get().success());
  runtime.stop();
}

TEST(DirectDriverRuntime, ModeTransitionTimeoutFailsAndReleasesGate) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_sport_status(0xB101U);  // never the target
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // The sport status never becomes the target, so the confirmation times
  // out after mode_timeout_ms and the future fails with the timeout code.
  std::future<Result> down = runtime.request_mode("down");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA103U); }));
  ASSERT_EQ(down.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(down.get().code, kSdkResTimeout);

  // The timed-out transition was completed, so a new request is accepted
  // instead of being rejected as pending.
  std::future<Result> stand = runtime.request_mode("stand");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA102U); }));
  sdk_ptr->set_sport_status(0xB102U);
  ASSERT_EQ(stand.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(stand.get().success());
  runtime.stop();
}

TEST(DirectDriverRuntime, ModeTimeoutSettlesAtDeadlineNotAtNextStatePoll) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_sport_status(0xB101U);  // never the target
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  config.mode_timeout_ms = 60ms;
  config.state_poll_period = 200ms;

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Align the mode request right after a state poll: with poll-lag
  // settlement the timeout would land at the first poll after the deadline
  // (~200ms away), and without the alignment the measured lag would depend
  // on where the request happened to fall inside the poll grid.
  const std::size_t polls_before = observer.state_count();
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot&) { return true; }, polls_before));

  // The sport status never becomes the target: the 60ms transition
  // deadline must settle the timeout itself instead of waiting for the
  // next state poll, which is 200ms away.
  std::future<Result> down = runtime.request_mode("down");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA103U); }));
  const auto armed = std::chrono::steady_clock::now();
  ASSERT_EQ(down.wait_for(2s), std::future_status::ready);
  const auto elapsed = std::chrono::steady_clock::now() - armed;
  EXPECT_EQ(down.get().code, kSdkResTimeout);

  // Bounded-error timing: settlement lands at the ~60ms deadline (same
  // order of magnitude), well below the 200ms poll period - a poll-lag
  // settlement would take ~200ms here.
  EXPECT_GE(elapsed, 40ms);
  EXPECT_LT(elapsed, 150ms);

  // The timed-out transition was completed, so a new request is accepted
  // instead of being rejected as pending. With the 200ms poll period the
  // stable status can never be observed inside the 60ms deadline, so the
  // fresh transition times out on its own - proving the gate released.
  std::future<Result> stand = runtime.request_mode("stand");
  EXPECT_NE(stand.wait_for(0ms), std::future_status::ready);
  ASSERT_EQ(stand.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(stand.get().code, kSdkResTimeout);
  runtime.stop();
}

TEST(DirectDriverRuntime, InvalidModeNameIsRejectedImmediately) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  std::future<Result> invalid = runtime.request_mode("fly");
  ASSERT_EQ(invalid.wait_for(0ms), std::future_status::ready);
  const Result result = invalid.get();
  EXPECT_EQ(result.code, kSdkResInvalidParam);
  EXPECT_NE(result.message.find("unknown robot mode"), std::string::npos);

  // Nothing reached the SDK and no transition was left pending.
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);
  std::future<Result> stand = runtime.request_mode("stand");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA102U); }));
  sdk_ptr->set_sport_status(0xB102U);
  ASSERT_EQ(stand.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(stand.get().success());
  runtime.stop();
}

// Submits a velocity while the runtime is live and waits for the rejection
// warning it must produce. The warning is recorded synchronously on the
// calling thread, so no further waiting is needed after this returns.
void expect_velocity_rejected(DirectDriverRuntime& runtime,
                              RecordingObserver& observer,
                              const std::string& reason_needle) {
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, reason_needle))
      << "expected a velocity rejection mentioning \"" << reason_needle
      << "\"";
}

TEST(DirectDriverRuntime, VelocityGateRejectsWithWarningAndNoMotion) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // 1. Not in the all-terrain Move state: rejected while the motion gate is
  // still open (checked before any fault or mode request latches it).
  sdk_ptr->set_snapshot([](SdkSnapshot& state) {
    state.sport_status = 0xB101U;
  });
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot& state) { return state.sport_status == 0xB101U; },
      observer.state_count()));
  expect_velocity_rejected(runtime, observer, "Move state");

  // 2. No control authority: rejected with a warning, nothing actuated.
  sdk_ptr->emit_status(true, false);
  ASSERT_TRUE(observer.wait_for_status(true, false));
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot& state) {
        return state.sdk_linked && !state.control_authority;
      },
      observer.state_count()));
  expect_velocity_rejected(runtime, observer, "authority");

  // 3. Robot system error: rejected, still nothing actuated.
  sdk_ptr->emit_status(true, true);
  ASSERT_TRUE(observer.wait_for_status(true, true));
  sdk_ptr->set_snapshot([](SdkSnapshot& state) { state.error_code = 1U; });
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot& state) { return state.error_code != 0U; },
      observer.state_count()));
  expect_velocity_rejected(runtime, observer, "system error");

  // 4. Pending mode transition: rejected.
  sdk_ptr->set_snapshot([](SdkSnapshot& state) {
    state.error_code = 0U;
    state.sport_status = 0xB104U;
  });
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot& state) {
        return state.error_code == 0U && state.sport_status == 0xB104U;
      },
      observer.state_count()));
  std::future<Result> stand = runtime.request_mode("stand");
  EXPECT_NE(stand.wait_for(0ms), std::future_status::ready);
  expect_velocity_rejected(runtime, observer, "mode transition");

  // None of the rejected submissions reached the SDK.
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);

  // 5. Emergency stop: the latch rejects further velocity, the pending mode
  // transition is cleared by the worker (future fails with the estop
  // reason), and zero velocity plus damping mode are executed. The stand
  // command must already be in flight so the ordering below is exact.
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA102U); }));
  std::future<Result> estop = runtime.trigger_estop(true);
  ASSERT_EQ(estop.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(estop.get().success());
  ASSERT_EQ(stand.wait_for(2s), std::future_status::ready);
  const Result stand_result = stand.get();
  EXPECT_FALSE(stand_result.success());
  EXPECT_NE(stand_result.message.find("emergency stop"), std::string::npos);
  expect_velocity_rejected(runtime, observer, "emergency stop");

  // The only motion that ever reached the SDK is the estop's zero; the only
  // mode commands are the stand request and the estop's damping.
  const std::size_t move_zero = first_call_index(sdk_ptr, [](const FakeAstrallSdk::Call& call) {
    return call.method == "move" && same_velocity(call.velocity, Velocity{});
  });
  const std::size_t stand_mode = first_call_index(sdk_ptr, [](const FakeAstrallSdk::Call& call) {
    return call.method == "set_mode" && call.mode == 0xA102U;
  });
  const std::size_t damping_mode = first_call_index(sdk_ptr, [](const FakeAstrallSdk::Call& call) {
    return call.method == "set_mode" && call.mode == 0xA101U;
  });
  EXPECT_EQ(sdk_ptr->call_count("move"), 1U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 2U);
  EXPECT_LT(stand_mode, move_zero);
  EXPECT_LT(move_zero, damping_mode);
  runtime.stop();

  // 6. Driver readiness not armed: a runtime that was never started rejects
  // velocity with a warning and makes no actuating call at all.
  auto cold_sdk = std::make_unique<FakeAstrallSdk>();
  auto cold_preflight = std::make_unique<FakeNetworkPreflight>();
  cold_preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver cold_observer;
  FakeAstrallSdk* cold_sdk_ptr = cold_sdk.get();
  DirectDriverRuntime cold_runtime(std::move(cold_sdk), std::move(cold_preflight),
                                   clock, fast_config(), &cold_observer);
  expect_velocity_rejected(cold_runtime, cold_observer, "readiness");
  EXPECT_EQ(cold_sdk_ptr->call_count("move"), 0U);
  EXPECT_EQ(cold_sdk_ptr->call_count("set_mode"), 0U);
  cold_runtime.stop();
}

TEST(DirectDriverRuntime, VelocityTickDispatchesClampedLatestValueAndDeadmanZeroes) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  OffsetClock clock;
  RuntimeConfig config = fast_config();
  config.motion_refresh_period = 20ms;
  config.deadman_ms = 100ms;

  FakeAstrallSdk* sdk_ptr = sdk.get();
  // Park the worker inside the first dispatched move() call: the parked
  // call already read the clamped value from the controller, so the
  // overwrite pair below is guaranteed to land before the worker's next
  // tick. No test sleeps and no assumption that both submissions fit into
  // one 20ms tick window.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool first_move_entered = false;
  bool release_first_move = false;
  std::atomic<bool> first_move{true};
  sdk_ptr->on_move_enter = [&] {
    if (!first_move.exchange(false)) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      first_move_entered = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_first_move; });
  };

  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // The 20ms tick loop forwards the accepted value, clamped to the
  // controller's limits; the worker is now parked inside that move() call.
  runtime.submit_velocity({2.0F, -2.0F, 0.5F});
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return first_move_entered; }));
  }

  // Latest-value overwrite: both submissions land while the worker is
  // parked, so the first of the pair can never be read by a tick.
  runtime.submit_velocity({0.3F, 0.2F, 0.1F});
  runtime.submit_velocity({0.9F, 0.8F, 0.7F});
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_first_move = true;
  }
  hook_cv.notify_all();

  // The parked dispatch completes with the clamped value; the next tick
  // dispatches only the latest of the overwrite pair.
  ASSERT_TRUE(
      wait_until([&] { return recorded_move(sdk_ptr, {1.0F, -1.0F, 0.5F}); }));
  ASSERT_TRUE(
      wait_until([&] { return recorded_move(sdk_ptr, {0.9F, 0.8F, 0.7F}); }));

  // The same latest value is re-dispatched on later ticks while it stays
  // within the command deadman.
  std::size_t latest_dispatches = 0;
  ASSERT_TRUE(wait_until([&] {
    latest_dispatches = 0;
    for (const FakeAstrallSdk::Call& call : sdk_ptr->calls()) {
      if (call.method == "move" &&
          same_velocity(call.velocity, {0.9F, 0.8F, 0.7F})) {
        ++latest_dispatches;
      }
    }
    return latest_dispatches >= 2U;
  }));

  // Deadman expiry: once the clock passes the deadman the tick loop sends
  // one zero-velocity move to stop the robot.
  clock.advance(101ms);
  ASSERT_TRUE(wait_until([&] {
    return recorded_move(sdk_ptr, Velocity{});
  }));

  // The overwritten value never reached the SDK, every move is one of the
  // three expected targets, and the final move is the deadman zero.
  const std::vector<FakeAstrallSdk::Call> calls = sdk_ptr->calls();
  bool saw_other_value = false;
  bool last_move_is_zero = true;
  bool saw_move = false;
  for (const FakeAstrallSdk::Call& call : calls) {
    if (call.method != "move") {
      continue;
    }
    saw_move = true;
    if (!same_velocity(call.velocity, {1.0F, -1.0F, 0.5F}) &&
        !same_velocity(call.velocity, {0.9F, 0.8F, 0.7F}) &&
        !same_velocity(call.velocity, Velocity{})) {
      saw_other_value = true;
    }
    last_move_is_zero = same_velocity(call.velocity, Velocity{});
  }
  EXPECT_FALSE(recorded_move(sdk_ptr, {0.3F, 0.2F, 0.1F}));
  EXPECT_FALSE(saw_other_value);
  EXPECT_TRUE(saw_move);
  EXPECT_TRUE(last_move_is_zero);
  EXPECT_GE(sdk_ptr->call_count("move"), 4U);
  runtime.stop();
}

TEST(DirectDriverRuntime, EstopLatchesClearsPendingAndDoesNotReplay) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  // No tick can fire during this test (the period exceeds the whole run),
  // so the assertions on what was never sent are deterministic. The long
  // mode timeout keeps the pending transition pending until the estop
  // clears it instead of timing out.
  config.motion_refresh_period = 30s;
  config.mode_timeout_ms = 10000ms;

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // An accepted velocity that must never be replayed after the stop, and a
  // mode transition that must be cleared by the estop.
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  std::future<Result> stand = runtime.request_mode("stand");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA102U); }));

  // Engage: latch, clear the pending transition, execute zero plus damping.
  std::future<Result> estop = runtime.trigger_estop(true);
  ASSERT_EQ(estop.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(estop.get().success());

  // The in-flight transition was aborted by the emergency stop, not by a
  // timeout.
  ASSERT_EQ(stand.wait_for(2s), std::future_status::ready);
  const Result stand_result = stand.get();
  EXPECT_FALSE(stand_result.success());
  EXPECT_NE(stand_result.message.find("emergency stop"), std::string::npos);

  // While latched, further velocity is rejected with a warning and never
  // reaches the SDK.
  runtime.submit_velocity({0.3F, 0.0F, 0.0F});
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, "emergency stop"));

  // Release the latch: immediate resolution, no replay of velocity or mode.
  std::future<Result> clear = runtime.trigger_estop(false);
  ASSERT_EQ(clear.wait_for(0ms), std::future_status::ready);
  EXPECT_TRUE(clear.get().success());

  // The loop stays alive (heartbeats continue) and neither the old velocity
  // nor the stand mode is ever replayed.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 3; }));
  EXPECT_EQ(sdk_ptr->call_count("move"), 1U);  // only the estop's zero
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 2U);  // stand + estop damping
  EXPECT_FALSE(recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}));
  EXPECT_FALSE(recorded_move(sdk_ptr, {0.3F, 0.0F, 0.0F}));

  // The gate re-opens after the release in the sense that the estop latch
  // no longer blocks: a fresh velocity is now rejected only by the motion
  // gate (the controller requires a fresh Move transition before motion
  // resumes), and a new mode request is no longer pending.
  runtime.submit_velocity({0.7F, 0.0F, 0.0F});
  EXPECT_EQ(observer.event_count(kRuntimeEventWarning, "velocity rejected"),
            2U);
  EXPECT_EQ(observer.event_count(kRuntimeEventWarning, "emergency stop"), 1U);
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, "velocity rejected: motion is gated"));
  std::future<Result> down = runtime.request_mode("down");
  ASSERT_TRUE(wait_until([&] { return recorded_set_mode(sdk_ptr, 0xA103U); }));
  sdk_ptr->set_sport_status(0xB103U);
  ASSERT_EQ(down.wait_for(2s), std::future_status::ready);
  EXPECT_TRUE(down.get().success());

  // Full actuation order: stand request, estop zero, estop damping, then
  // the post-clear down request.
  const std::size_t stand_index = first_call_index(sdk_ptr,
      [](const FakeAstrallSdk::Call& call) {
        return call.method == "set_mode" && call.mode == 0xA102U;
      });
  const std::size_t zero_index = first_call_index(sdk_ptr,
      [](const FakeAstrallSdk::Call& call) {
        return call.method == "move" && same_velocity(call.velocity, Velocity{});
      });
  const std::size_t damping_index = first_call_index(sdk_ptr,
      [](const FakeAstrallSdk::Call& call) {
        return call.method == "set_mode" && call.mode == 0xA101U;
      });
  const std::size_t down_index = first_call_index(sdk_ptr,
      [](const FakeAstrallSdk::Call& call) {
        return call.method == "set_mode" && call.mode == 0xA103U;
      });
  EXPECT_LT(stand_index, zero_index);
  EXPECT_LT(zero_index, damping_index);
  EXPECT_LT(damping_index, down_index);
  runtime.stop();
}

TEST(DirectDriverRuntime, StopWithAuthoritySendsZeroAndDamping) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // stop() joins the worker, so once it returns the whole actuation and
  // teardown history is final.
  runtime.stop();
  EXPECT_EQ(sdk_ptr->call_count("move"), 1U);
  EXPECT_TRUE(recorded_move(sdk_ptr, Velocity{}));
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 1U);
  EXPECT_TRUE(recorded_set_mode(sdk_ptr, 0xA101U));
  EXPECT_EQ(sdk_ptr->call_count("request_authority"), 0U);

  // Order: zero velocity, then damping mode, then the teardown deinit.
  const std::size_t zero_index = first_call_index(sdk_ptr,
      [](const FakeAstrallSdk::Call& call) {
        return call.method == "move" && same_velocity(call.velocity, Velocity{});
      });
  const std::size_t damping_index = first_call_index(sdk_ptr,
      [](const FakeAstrallSdk::Call& call) {
        return call.method == "set_mode" && call.mode == 0xA101U;
      });
  const std::size_t deinit_index = first_call_index(sdk_ptr,
      [](const FakeAstrallSdk::Call& call) { return call.method == "deinit"; });
  EXPECT_LT(zero_index, damping_index);
  EXPECT_LT(damping_index, deinit_index);
}

TEST(DirectDriverRuntime, StopWithoutAuthorityNeverActuates) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  sdk_ptr->emit_status(true, false);
  ASSERT_TRUE(observer.wait_for_status(true, false));
  ASSERT_TRUE(observer.wait_for_state_matching([](const SdkSnapshot& state) {
    return state.sdk_linked && !state.control_authority;
  }));

  runtime.stop();
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("request_authority"), 0U);
  EXPECT_GE(sdk_ptr->call_count("deinit"), 1U);
}

TEST(DirectDriverRuntime, StopFromInflightBlockingCallbackDoesNotDeadlock) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Instrument the fake so the first move call - which runs on the worker
  // thread - calls stop() from inside the in-flight SDK call and then
  // blocks until the test releases it.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool stop_returned = false;
  bool release_hook = false;
  std::atomic<bool> first_move{true};
  sdk_ptr->on_move_enter = [&] {
    if (!first_move.exchange(false)) {
      return;
    }
    runtime.stop();  // worker thread: must flag shutdown, not join itself
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      stop_returned = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_hook; });
  };

  runtime.submit_velocity({0.5F, 0.0F, 0.0F});

  // Event-driven: stop() returned from inside the callback. A self-join
  // deadlock would time this wait out.
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return stop_returned; }));
  }
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_hook = true;
  }
  hook_cv.notify_all();

  // The worker then exits: the session is torn down and deinitialized, and
  // the actuated stop (zero velocity plus damping) still ran.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));
  EXPECT_TRUE(recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}));
  EXPECT_TRUE(recorded_move(sdk_ptr, Velocity{}));
  EXPECT_TRUE(recorded_set_mode(sdk_ptr, 0xA101U));
}

// Observer that records like RecordingObserver and additionally calls
// stop() from inside the first on_status callback that reports a lost
// control authority, simulating a node shutting down from its own status
// callback (dispatched on the fake's "vendor" thread).
class StopOnAuthorityLossObserver final : public RecordingObserver {
 public:
  explicit StopOnAuthorityLossObserver(DirectDriverRuntime* runtime)
      : runtime_(runtime) {}
  void set_runtime(DirectDriverRuntime* runtime) { runtime_ = runtime; }
  void on_status(bool linked, bool control_authority) override {
    RecordingObserver::on_status(linked, control_authority);
    if (!control_authority && !fired_.exchange(true)) {
      runtime_->stop();  // inside an observer callback: must not join
      stop_returned_.store(true);
    }
  }
  std::atomic<bool> stop_returned_{false};

 private:
  DirectDriverRuntime* runtime_{nullptr};
  std::atomic<bool> fired_{false};
};

// Observer that records like RecordingObserver and, on the first on_status
// callback (dispatched on a "vendor" thread), synchronously triggers an
// inner observer callback (a velocity-rejection on_event) and then calls
// stop() from inside the outer callback. A non-reentrant callback guard
// would let the inner dispatch clear the in-observer flag, after which
// stop() would misjudge the outer callback as exited and join the worker
// from the vendor thread - deadlock against the parked worker.
class NestedCallbackStopObserver final : public RecordingObserver {
 public:
  explicit NestedCallbackStopObserver(DirectDriverRuntime* runtime)
      : runtime_(runtime) {}
  void set_runtime(DirectDriverRuntime* runtime) { runtime_ = runtime; }
  // Arm the stop behavior only after the test setup is complete, so the
  // connect_live status report (delivered on the test thread) is recorded
  // without triggering it.
  void arm() { armed_.store(true); }
  void on_status(bool linked, bool control_authority) override {
    RecordingObserver::on_status(linked, control_authority);
    if (!armed_.load()) {
      return;
    }
    if (fired_.exchange(false)) {
      // Outer callback on the vendor thread: synchronously dispatch an
      // inner observer callback, then stop() from inside the outer frame.
      runtime_->submit_velocity({0.7F, 0.0F, 0.0F});  // rejected -> on_event
      runtime_->stop();
      stop_returned_.store(true);
    }
  }
  std::atomic<bool> stop_returned_{false};

 private:
  DirectDriverRuntime* runtime_{nullptr};
  std::atomic<bool> fired_{true};
  std::atomic<bool> armed_{false};
};

TEST(DirectDriverRuntime, VelocityRejectedAfterReconnectBeforeFirstSnapshot) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  // Session 1: a live link with authority in the all-terrain Move state.
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Break the session through the status callback. From here on every
  // snapshot() parks the worker, so the reconnected session can be
  // observed strictly before its first snapshot is delivered.
  sdk_ptr->set_snapshot_blocked(true);
  sdk_ptr->emit_status(false, false);
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected", 2));

  // Reconnected, but no snapshot of the new session arrived yet: the
  // session-1 link/authority/Move state must not re-arm the driver. The
  // gate must report the missing readiness, not a stale live state.
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, "driver readiness"));

  // The first snapshot of the new session re-arms the gate and restores a
  // live state; nothing was ever actuated.
  sdk_ptr->set_snapshot([](SdkSnapshot& state) {
    state.sdk_linked = true;
    state.control_authority = true;
    state.sport_status = 0xB104U;
  });
  sdk_ptr->set_snapshot_blocked(false);
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot& state) {
        return state.sdk_linked && state.control_authority;
      },
      observer.state_count()));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 2; }));
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  runtime.stop();
}

TEST(DirectDriverRuntime, VelocityRejectedInReconnectWindowAfterHeartbeatFailure) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  // Session 1: a live link with authority in the all-terrain Move state.
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // A velocity is accepted and dispatched by the motion tick loop.
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  ASSERT_TRUE(
      wait_until([&] { return recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}); }));

  // The single heartbeat failure ends the session (default threshold).
  // Park every snapshot() once the session is over, so the reconnected
  // session can be observed strictly before its first snapshot re-arms the
  // driver readiness gate. Heartbeats must succeed again from here on:
  // the reconnected session's heartbeat fires before its first poll and
  // would otherwise end the session before the gate can re-arm.
  sdk_ptr->next_heartbeat =
      Result::failure(kSdkResTimeout, "test-injected heartbeat timeout");
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "heartbeat"));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));
  sdk_ptr->next_heartbeat = Result::ok();
  sdk_ptr->set_snapshot_blocked(true);
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected", 2));

  // Reconnected, but no snapshot of the new session arrived yet: the
  // first-snapshot gate keeps motion closed and the submission is dropped
  // with a readiness warning, never reaching the SDK.
  const std::size_t moves_before = sdk_ptr->call_count("move");
  runtime.submit_velocity({0.7F, 0.0F, 0.0F});
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, "driver readiness"));
  EXPECT_EQ(sdk_ptr->call_count("move"), moves_before);

  // The first snapshot of the new session re-arms the gate; the rejected
  // submission is never replayed afterwards. The poll baseline is captured
  // before the unblock: the worker is parked in the first poll, so the
  // baseline deterministically excludes the snapshot that follows.
  const std::size_t polls_before = observer.state_count();
  sdk_ptr->set_snapshot_blocked(false);
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot& state) {
        return state.sdk_linked && state.control_authority;
      },
      polls_before));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 3; }));
  EXPECT_FALSE(recorded_move(sdk_ptr, {0.7F, 0.0F, 0.0F}));
  runtime.stop();
}

TEST(DirectDriverRuntime, CommandsRejectedBeforeNewSessionsStatusCallback) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  // Session 1: a live link with control authority.
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // End session 1 through the heartbeat path: no status callback clears
  // the runtime-side link/authority flags, so any stale true/true left by
  // the session teardown is observable after the reconnect.
  sdk_ptr->next_heartbeat =
      Result::failure(kSdkResTimeout, "test-injected heartbeat timeout");
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "heartbeat"));
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));
  sdk_ptr->next_heartbeat = Result::ok();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected", 2));

  // Reconnected, but the new session's first status callback has not been
  // delivered: commands must be refused immediately instead of being
  // forwarded under the previous session's authority.
  std::future<Result> authority = runtime.request_authority(true);
  ASSERT_EQ(authority.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(authority.get().code, kSdkResNotInit);
  std::future<Result> mode = runtime.request_mode("stand");
  ASSERT_EQ(mode.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(mode.get().code, kSdkResNotInit);
  EXPECT_EQ(sdk_ptr->call_count("request_authority"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);

  // The software emergency stop latches locally but is answered
  // immediately with no SDK call.
  std::future<Result> estop = runtime.trigger_estop(true);
  ASSERT_EQ(estop.wait_for(0ms), std::future_status::ready);
  EXPECT_EQ(estop.get().code, kSdkResNotInit);
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);

  // stop() must not actuate either: the new session has no confirmed link
  // or authority, so no zero-velocity or damping call may be made.
  runtime.stop();
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);
  EXPECT_GE(sdk_ptr->call_count("deinit"), 2U);
}

TEST(DirectDriverRuntime, AuthorityLossCallbackStopsMotionImmediately) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  // A long poll period proves the gate closes through the status callback
  // itself, not through a state poll (the next one is 500ms away).
  config.state_poll_period = 500ms;

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // A velocity is accepted and dispatched by the motion tick loop...
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  ASSERT_TRUE(
      wait_until([&] { return recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}); }));

  // ...when the SDK reports the authority loss through the status callback.
  // The callback closes the safety gate synchronously and requests an
  // immediate motion tick, so the zero reaches the SDK without waiting for
  // the next state poll.
  sdk_ptr->emit_status(true, false);
  ASSERT_TRUE(observer.wait_for_status(true, false));
  ASSERT_TRUE(wait_until([&] { return recorded_move(sdk_ptr, Velocity{}); }));

  // New velocity is rejected immediately with the authority reason and
  // never reaches the SDK; the last move ever sent is the stop zero.
  runtime.submit_velocity({0.3F, 0.0F, 0.0F});
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, "authority"));
  ASSERT_TRUE(
      wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 3; }));
  EXPECT_FALSE(recorded_move(sdk_ptr, {0.3F, 0.0F, 0.0F}));
  bool saw_move = false;
  bool last_move_is_zero = true;
  for (const FakeAstrallSdk::Call& call : sdk_ptr->calls()) {
    if (call.method != "move") {
      continue;
    }
    saw_move = true;
    EXPECT_TRUE(same_velocity(call.velocity, {0.5F, 0.0F, 0.0F}) ||
                same_velocity(call.velocity, Velocity{}))
        << "unexpected move target reached the SDK";
    last_move_is_zero = same_velocity(call.velocity, Velocity{});
  }
  EXPECT_TRUE(saw_move);
  EXPECT_TRUE(last_move_is_zero);
  runtime.stop();
}

TEST(DirectDriverRuntime, StaleSnapshotCannotOverrideNewerAuthorityLoss) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  // Park the worker inside a state poll, then deliver the authority-loss
  // status callback while the poll is in flight: the parked snapshot still
  // carries the session's old authority=true, and the fake re-stales it
  // explicitly before the release. The controller link/authority must come
  // from the callback cache, never from such an older snapshot.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool snapshot_entered = false;
  sdk_ptr->on_snapshot_enter = [&] {
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      snapshot_entered = true;
    }
    hook_cv.notify_all();
  };

  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  sdk_ptr->set_snapshot_blocked(true);
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return snapshot_entered; }));
  }

  // Newer truth: the SDK reports the authority loss through the status
  // callback while the poll is parked.
  sdk_ptr->emit_status(true, false);
  ASSERT_TRUE(observer.wait_for_status(true, false));
  // Make the snapshot the parked poll will return stale again: it claims
  // the authority that the callback just revoked.
  sdk_ptr->set_snapshot([](SdkSnapshot& state) {
    state.sdk_linked = true;
    state.control_authority = true;
    state.sport_status = 0xB104U;
  });

  // Release the poll and wait until it was applied (the observer receives
  // the raw snapshot at the end of the poll branch, after the controller
  // update).
  const std::size_t polls_before = observer.state_count();
  sdk_ptr->set_snapshot_blocked(false);
  ASSERT_TRUE(observer.wait_for_new_state_matching(
      [](const SdkSnapshot&) { return true; }, polls_before));

  // The controller state must still report the revoked authority: the new
  // velocity is rejected and nothing reaches the SDK.
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, "authority"));
  EXPECT_FALSE(recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}));
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  runtime.stop();
}

TEST(DirectDriverRuntime, LinkDropDuringInitEndsSessionEvenAfterRelink) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  // The SDK reports the link drop and then re-links synchronously inside
  // the same init(): the drop must latch the session end, so the later
  // linked=true report cannot keep the session alive.
  sdk_ptr->on_init_enter = [&] {
    sdk_ptr->emit_status(false, false);
    sdk_ptr->emit_status(true, true);
  };
  runtime.start();

  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  // The latched drop ends the session immediately: no heartbeat ever runs,
  // and the worker retries from preflight.
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));
  EXPECT_EQ(sdk_ptr->call_count("heartbeat"), 0U);
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("init") >= 2; }));
  runtime.stop();
}

TEST(DirectDriverRuntime, StopFromObserverCallbackOnVendorThreadDoesNotDeadlock) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  StopOnAuthorityLossObserver observer(nullptr);
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  observer.set_runtime(&runtime);

  // Park the worker inside an in-flight move() call: the vendor status
  // callback that follows can only complete when the worker returns from
  // the SDK, so stop() must never join the worker from that callback.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  std::atomic<bool> worker_in_move{false};
  bool release_worker = false;
  sdk_ptr->on_move_enter = [&] {
    worker_in_move.store(true, std::memory_order_relaxed);
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_worker; });
  };

  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  ASSERT_TRUE(wait_until([&] { return worker_in_move.load(); }));

  // A "vendor" thread delivers the status callback; the observer calls
  // stop() from inside it. A join from that thread would deadlock against
  // the worker parked in move(), so the bounded wait below proves stop()
  // returned. The hook is released and the thread joined even on failure,
  // so a RED run cannot abort the binary with an unjoined thread.
  std::thread vendor_thread([&] { sdk_ptr->emit_status(true, false); });
  const bool stop_returned_seen =
      wait_until([&] { return observer.stop_returned_.load(); });
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_worker = true;
  }
  hook_cv.notify_all();
  vendor_thread.join();
  ASSERT_TRUE(stop_returned_seen);

  // The worker finishes its in-flight SDK call and completes the teardown
  // on its own.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));
  EXPECT_TRUE(recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}));
}

TEST(DirectDriverRuntime, ConcurrentStopIsSafe) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Park the worker inside an in-flight move() call so the first stopper
  // must wait for it while the second stopper races the join.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool worker_in_move = false;
  bool release_worker = false;
  sdk_ptr->on_move_enter = [&] {
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      worker_in_move = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_worker; });
  };
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return worker_in_move; }));
  }

  // Two plain threads call stop() concurrently, released together, and the
  // worker is only released once both are inside stop(): exactly one join
  // must happen and both callers must return.
  std::atomic<int> ready{0};
  std::atomic<bool> go{false};
  std::atomic<int> entered{0};
  std::atomic<int> returned{0};
  auto stopper = [&] {
    ready.fetch_add(1);
    while (!go.load()) {
      std::this_thread::yield();
    }
    entered.fetch_add(1);
    runtime.stop();
    returned.fetch_add(1);
  };
  std::thread stopper_a(stopper);
  std::thread stopper_b(stopper);
  while (ready.load() < 2) {
    std::this_thread::yield();
  }
  go.store(true);
  while (entered.load() < 2) {
    std::this_thread::yield();
  }
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_worker = true;
  }
  hook_cv.notify_all();
  stopper_a.join();
  stopper_b.join();
  EXPECT_EQ(returned.load(), 2);

  // The single join completed the teardown: the worker is gone and the SDK
  // was deinitialized exactly through the normal session path.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));
  EXPECT_TRUE(recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}));
}

TEST(DirectDriverRuntime, StopFromNestedObserverCallbackDoesNotJoinWorker) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  NestedCallbackStopObserver observer(nullptr);
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  observer.set_runtime(&runtime);

  // Park the worker inside an in-flight move() call: joining it from the
  // vendor thread would deadlock until the hook is released.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool worker_in_move = false;
  bool release_worker = false;
  sdk_ptr->on_move_enter = [&] {
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      worker_in_move = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_worker; });
  };

  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return worker_in_move; }));
  }

  // Arm the nested stop behavior only now: the vendor thread's status
  // callback must be the one that triggers it.
  observer.arm();

  // The vendor thread delivers a status callback; the observer's outer
  // on_status synchronously triggers the inner on_event (the velocity
  // rejection) and then calls stop() from inside the outer callback. A
  // non-reentrant guard would clear the in-observer flag on the inner
  // dispatch and stop() would join the parked worker from the vendor
  // thread; the bounded wait below would time out.
  std::thread vendor_thread([&] { sdk_ptr->emit_status(true, false); });
  const bool stop_returned_seen =
      wait_until([&] { return observer.stop_returned_.load(); });
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_worker = true;
  }
  hook_cv.notify_all();
  vendor_thread.join();
  ASSERT_TRUE(stop_returned_seen);

  // The worker finishes its in-flight SDK call and completes the teardown
  // on its own; the inner callback was dispatched and recorded.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));
  EXPECT_TRUE(recorded_move(sdk_ptr, {0.5F, 0.0F, 0.0F}));
  EXPECT_EQ(observer.event_count(kRuntimeEventWarning, "velocity rejected"),
            1U);
}

TEST(DirectDriverRuntime, ThreeWayStopDeadlockIsBrokenByCallbackEarlyReturn) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  StopOnAuthorityLossObserver observer(nullptr);
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  observer.set_runtime(&runtime);

  // Park the worker inside an in-flight move() call.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool worker_in_move = false;
  bool release_worker = false;
  sdk_ptr->on_move_enter = [&] {
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      worker_in_move = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_worker; });
  };

  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));
  runtime.submit_velocity({0.5F, 0.0F, 0.0F});
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return worker_in_move; }));
  }

  // Plain thread A enters stop(): it must be blocked in join while holding
  // stop_mutex_ (the worker is parked in move()). Once A is provably inside
  // stop and not returned, a vendor status callback arrives whose observer
  // calls stop() again: that callback-side stop() must return without ever
  // taking stop_mutex_, or the three-way wait deadlocks and the bounded
  // wait below times out.
  std::atomic<bool> in_stop{false};
  std::atomic<bool> stop_returned{false};
  std::thread stopper_a([&] {
    in_stop.store(true);
    runtime.stop();
    stop_returned.store(true);
  });
  ASSERT_TRUE(wait_until([&] {
    return in_stop.load() && !stop_returned.load();
  }));

  std::thread vendor_thread([&] { sdk_ptr->emit_status(true, false); });
  const bool callback_stop_returned =
      wait_until([&] { return observer.stop_returned_.load(); });
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_worker = true;
  }
  hook_cv.notify_all();
  vendor_thread.join();
  stopper_a.join();
  ASSERT_TRUE(callback_stop_returned);
  ASSERT_TRUE(stop_returned.load());
  EXPECT_TRUE(sdk_ptr->call_count("deinit") >= 1);
}

TEST(DirectDriverRuntime, LatchedLinkDropRejectsCommandsAndStopActuation) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  // The SDK drops the link and re-links inside init(): the latch marks the
  // session dead even though the level is back to linked.
  sdk_ptr->on_init_enter = [&] {
    sdk_ptr->emit_status(false, false);
    sdk_ptr->emit_status(true, true);
  };
  // Park the worker at the first state poll so the window between "session
  // connected" and "worker observes the latch" is under test control.
  sdk_ptr->set_snapshot_blocked(true);
  runtime.start();
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));

  // Commands must be rejected immediately by the latch, not just by the
  // eventual session teardown, and the software emergency stop must stay
  // local-only with zero SDK calls.
  std::future<Result> authority = runtime.request_authority(true);
  ASSERT_EQ(authority.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(authority.get().code, kSdkResNotInit);
  std::future<Result> estop = runtime.trigger_estop(true);
  // The latch must reject the emergency-stop request immediately, not via a
  // future that only resolves during the worker's teardown.
  ASSERT_EQ(estop.wait_for(0s), std::future_status::ready);
  EXPECT_FALSE(estop.get().success());
  EXPECT_EQ(sdk_ptr->call_count("request_authority"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);

  // Release the parked poll and stop: the session was latched dead, so the
  // shutdown path must not actuate either.
  sdk_ptr->set_snapshot_blocked(false);
  runtime.stop();
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  EXPECT_EQ(sdk_ptr->call_count("set_mode"), 0U);
}

TEST(DirectDriverRuntime, ConcurrentStartStopNeverRacesTheWorkerThread) {
  for (int iteration = 0; iteration < 30; ++iteration) {
    auto sdk = std::make_unique<FakeAstrallSdk>();
    auto preflight = std::make_unique<FakeNetworkPreflight>();
    preflight->next_decision = NetworkDecision{true, "test network ready"};
    RecordingObserver observer;
    SteadyMonotonicClock clock;
    RuntimeConfig config = fast_config();
    DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                                config, &observer);

    std::atomic<bool> go{false};
    std::atomic<int> ready{0};
    std::thread starter([&] {
      ready.fetch_add(1);
      while (!go.load()) {
        std::this_thread::yield();
      }
      runtime.start();
    });
    std::thread stopper([&] {
      ready.fetch_add(1);
      while (!go.load()) {
        std::this_thread::yield();
      }
      runtime.stop();
    });
    while (ready.load() < 2) {
      std::this_thread::yield();
    }
    go.store(true);
    starter.join();
    stopper.join();
  }
}

TEST(DirectDriverRuntime, AuthorityLossIsNotResurrectedByConcurrentPolls) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Deliver an authority loss and race it against the periodic state polls
  // that still report the old authority=true snapshot. The structural
  // guarantee is that the poll composes and applies the controller status
  // inside one wake_mutex_ critical section, so the loss is never
  // overwritten by a stale poll; this loop stresses the interleaving.
  for (int round = 0; round < 20; ++round) {
    sdk_ptr->emit_status(true, false);
  }
  const std::size_t moves_before = sdk_ptr->call_count("move");
  runtime.submit_velocity({0.3F, 0.0F, 0.0F});
  EXPECT_TRUE(observer.has_event(kRuntimeEventWarning, "authority"));
  ASSERT_TRUE(wait_until([&] {
    return observer.has_event(kRuntimeEventWarning, "authority");
  }));
  EXPECT_EQ(sdk_ptr->call_count("move"), moves_before);
  runtime.stop();
}

TEST(DirectDriverRuntime, QueuedCommandNeverExecutesAfterLinkDrop) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Park the worker inside the next snapshot() so a command can be queued
  // while the worker is unable to consume it.
  std::mutex hook_mutex;
  std::condition_variable hook_cv;
  bool worker_in_snapshot = false;
  bool release_worker = false;
  sdk_ptr->on_snapshot_enter = [&] {
    {
      std::lock_guard<std::mutex> lock(hook_mutex);
      worker_in_snapshot = true;
    }
    hook_cv.notify_all();
    std::unique_lock<std::mutex> lock(hook_mutex);
    hook_cv.wait(lock, [&] { return release_worker; });
  };
  sdk_ptr->set_snapshot_blocked(true);
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_cv.wait_for(lock, 2s, [&] { return worker_in_snapshot; }));
  }

  // Queue a command while the worker is parked, then latch a link drop. The
  // command must never reach the SDK: the worker re-checks the latch after
  // every wait and bails to the teardown path.
  std::future<Result> authority = runtime.request_authority(true);
  sdk_ptr->emit_status(false, false);

  // Release the parked worker BEFORE waiting for the event: "connection
  // lost" is only emitted by the teardown that runs after the worker leaves
  // the blocked snapshot(), so waiting while the worker is still parked
  // could hang this test on an assertion failure (the destructor would join
  // a worker that is never released).
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_worker = true;
  }
  hook_cv.notify_all();
  sdk_ptr->set_snapshot_blocked(false);

  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"));
  ASSERT_EQ(authority.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(authority.get().code, kSdkResNotInit);
  EXPECT_EQ(sdk_ptr->call_count("request_authority"), 0U);
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));
  runtime.stop();
}

TEST(DirectDriverRuntime, ConcurrentStatusCallbacksCannotResurrectAuthority) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  RecordingObserver observer;
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();

  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();
  ASSERT_TRUE(connect_live(runtime, observer, sdk_ptr));

  // Two concurrent vendor callbacks: one keeps re-reporting authority, the
  // other drops it. Both the cache write and the controller update happen
  // in one wake_mutex_ critical section, so an older report can never
  // resurrect authority after a newer report dropped it.
  std::atomic<bool> go{false};
  std::atomic<int> ready{0};
  auto authority_dropper = [&] {
    ready.fetch_add(1);
    while (!go.load()) {
      std::this_thread::yield();
    }
    for (int round = 0; round < 50; ++round) {
      sdk_ptr->emit_status(true, false);
    }
  };
  auto authority_keeper = [&] {
    ready.fetch_add(1);
    while (!go.load()) {
      std::this_thread::yield();
    }
    for (int round = 0; round < 50; ++round) {
      sdk_ptr->emit_status(true, true);
    }
  };
  std::thread dropper(authority_dropper);
  std::thread keeper(authority_keeper);
  while (ready.load() < 2) {
    std::this_thread::yield();
  }
  go.store(true);
  dropper.join();
  keeper.join();

  // The last report wins; a final drop settles the gate closed and the
  // worker's immediate motion tick stops any dispatched velocity. A single
  // final drop must close the gate regardless of the interleaving above.
  sdk_ptr->emit_status(true, false);
  runtime.submit_velocity({0.3F, 0.0F, 0.0F});
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventWarning, "authority"));
  EXPECT_EQ(sdk_ptr->call_count("move"), 0U);
  runtime.stop();
}

// ---------------------------------------------------------------------------
// F4: A throwing observer is isolated by dispatch_observer_safely; the
// exception is caught and counted, never escapes into the worker or a caller,
// and never calls back into the observer. The worker keeps running and the
// non-throwing observer callbacks are still delivered.
// ---------------------------------------------------------------------------

// (a) on_event throws on every dispatch: the worker survives, the session is
// established, the error is counted, the rejecting run keeps working, and
// stop() is clean (no terminate, no deadlock).
TEST(DirectDriverRuntime, F4_ThrowingObserverOnEventWorkerSurvives) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  ThrowingObserver observer(ThrowingObserver::kOnEvent);
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  // "connected" is delivered through notify_event -> on_event, which throws
  // and is swallowed. The session must still be established (heartbeat and
  // state polls advance) and the throws must be counted.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));
  ASSERT_TRUE(wait_until([&] { return observer.state_count() >= 1; }));
  ASSERT_TRUE(wait_until([&] { return runtime.observer_error_count() >= 1; }))
      << "the throwing on_event was not caught and counted";
  // The run keeps alive: heartbeat continues to grow, non-throwing on_state
  // keeps being recorded.
  const std::size_t hb_before = sdk_ptr->call_count("heartbeat");
  ASSERT_TRUE(wait_until([&] {
    return sdk_ptr->call_count("heartbeat") >= hb_before + 3;
  })) << "worker died under a throwing on_event";
  ASSERT_TRUE(wait_until([&] { return observer.state_count() >= 3; }));
  // Non-throwing callbacks (on_status) still arrive and are recorded even
  // though on_event keeps throwing.
  sdk_ptr->emit_status(true, true);
  ASSERT_TRUE(observer.wait_for_status(true, true));
  runtime.stop();  // must return cleanly (no terminate, no deadlock)
}

// (b) on_state throws on every state poll: the worker's loop is not killed,
// heartbeats keep advancing, and the throws are counted.
TEST(DirectDriverRuntime, F4_ThrowingObserverOnStateHeartbeatAdvances) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  ThrowingObserver observer(ThrowingObserver::kOnState);
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  // on_state throws+is caught; count grows and heartbeats keep advancing.
  ASSERT_TRUE(wait_until([&] { return runtime.observer_error_count() >= 1; }));
  const std::size_t hb_before = sdk_ptr->call_count("heartbeat");
  ASSERT_TRUE(wait_until([&] {
    return sdk_ptr->call_count("heartbeat") >= hb_before + 3;
  })) << "worker loop died under a throwing on_state";
  // on_event is not throwing here, so lifecycle events are still recorded.
  EXPECT_GE(runtime.observer_error_count(), 1u);
  runtime.stop();
}

// (c) on_status throws: a link drop still ends the session normally (the
// status callback latches the drop and emits "connection lost" before the
// throw is swallowed).
TEST(DirectDriverRuntime, F4_ThrowingObserverOnStatusLinkDropEndsSession) {
  auto sdk = std::make_unique<FakeAstrallSdk>();
  sdk->set_linked(true);
  sdk->set_authority(true);
  sdk->set_sport_status(0xB104U);
  auto preflight = std::make_unique<FakeNetworkPreflight>();
  preflight->next_decision = NetworkDecision{true, "test network ready"};
  ThrowingObserver observer(ThrowingObserver::kOnStatus);
  SteadyMonotonicClock clock;
  RuntimeConfig config = fast_config();
  FakeAstrallSdk* sdk_ptr = sdk.get();
  DirectDriverRuntime runtime(std::move(sdk), std::move(preflight), clock,
                              config, &observer);
  runtime.start();

  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventInfo, "connected"));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("heartbeat") >= 1; }));
  // Non-throwing on_state is still recorded even with a throwing on_status.
  ASSERT_TRUE(wait_until([&] { return observer.state_count() >= 1; }));

  // A link drop flows through on_status (throws) and notify_event (records).
  sdk_ptr->emit_status(false, false);
  ASSERT_TRUE(wait_until([&] { return runtime.observer_error_count() >= 1; }))
      << "throwing on_status was not caught and counted";
  ASSERT_TRUE(observer.wait_for_event(kRuntimeEventError, "connection lost"))
      << "link drop did not end the session despite the throwing on_status";
  // Session teardown happened: deinit ran and reconnect re-inited.
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("deinit") >= 1; }));
  ASSERT_TRUE(wait_until([&] { return sdk_ptr->call_count("init") >= 2; }));
  runtime.stop();
}

}  // namespace
}  // namespace hypertron_ros2_bridge
