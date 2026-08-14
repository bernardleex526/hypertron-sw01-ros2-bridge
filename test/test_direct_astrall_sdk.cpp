#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "fake_astrall_vendor_api.hpp"
#include "hypertron_ros2_bridge/astrall_sdk.hpp"
#include "hypertron_ros2_bridge/astrall_vendor_api.hpp"

namespace hypertron_ros2_bridge {
namespace {

using test::FakeAstrallVendorApi;
using test::make_imu_buffer;
using test::make_sport_buffer;

SdkCallbacks recording_callbacks(std::vector<bool>* links,
                                std::vector<bool>* authorities) {
  SdkCallbacks callbacks;
  callbacks.on_status = [links, authorities](bool linked, bool authority) {
    if (links) {
      links->push_back(linked);
    }
    if (authorities) {
      authorities->push_back(authority);
    }
  };
  return callbacks;
}

bool wait_until(const std::function<bool()>& condition,
                std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return condition();
}

const std::vector<std::uint8_t> kImuFrame = make_imu_buffer(
    7, {1.0F, 2.0F, 3.0F}, {0.1F, 0.2F, 0.3F}, {0.0F, 0.0F, 0.0F, 1.0F},
    0.25F, -0.1F, 1.5F, 0.5F, -0.5F);

const std::vector<std::uint8_t> kSportFrame =
    make_sport_buffer(8, {0.1F, 0.2F, 0.3F, 0.4F});

TEST(DirectAstrallSdk, RejectsShortAndEmptyStatusBuffersWithoutSideEffects) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  std::vector<bool> links;
  std::vector<bool> authorities;
  EXPECT_TRUE(sdk.init(recording_callbacks(&links, &authorities), 60000).success());

  // Short packet (non-null, len 0) and empty packet (null, len 0): neither
  // may update cached state nor invoke the project callback.
  vendor.emit_status_short();
  vendor.emit_status_empty();
  EXPECT_TRUE(links.empty());
  EXPECT_TRUE(authorities.empty());
  const SdkSnapshot before = sdk.snapshot();
  EXPECT_FALSE(before.sdk_linked);
  EXPECT_FALSE(before.control_authority);

  // A valid 1-byte status reaches both cache and callback.
  vendor.emit_status(0x01U);  // bit0=link, bit1=ctrlAuthority
  ASSERT_EQ(links.size(), 1U);
  EXPECT_TRUE(links[0]);
  EXPECT_FALSE(authorities[0]);
  const SdkSnapshot after = sdk.snapshot();
  EXPECT_TRUE(after.sdk_linked);
  EXPECT_FALSE(after.control_authority);
}

TEST(DirectAstrallSdk, KeepsStatusDeliveredSynchronouslyDuringInit) {
  FakeAstrallVendorApi vendor;
  vendor.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  vendor.state_->init_status_byte = 0x01U;  // linked, no authority
  DirectAstrallSdk sdk(vendor.api());
  std::vector<bool> links;
  std::vector<bool> authorities;
  const Result result =
      sdk.init(recording_callbacks(&links, &authorities), 60000);
  EXPECT_TRUE(result.success());
  // The synchronous status must reach the project callback...
  ASSERT_EQ(links.size(), 1U);
  EXPECT_TRUE(links[0]);
  EXPECT_FALSE(authorities[0]);
  // ...and its values must be retained, not reset, after init completes.
  const SdkSnapshot snapshot = sdk.snapshot();
  EXPECT_TRUE(snapshot.sdk_linked);
  EXPECT_FALSE(snapshot.control_authority);
}

TEST(DirectAstrallSdk, DuplicateInitFailsAndCallsVendorInitOnce) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
  const Result second = sdk.init(SdkCallbacks{}, 60000);
  EXPECT_FALSE(second.success());
  EXPECT_NE(second.message.find("already initialized"), std::string::npos);
  EXPECT_EQ(vendor.call_count("init"), 1U);
}

TEST(DirectAstrallSdk, FailedInitClearsStateAndCallbacks) {
  FakeAstrallVendorApi vendor;
  vendor.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  vendor.state_->init_status_byte = 0x03U;
  vendor.state_->next_init = kSdkResFailed;
  DirectAstrallSdk sdk(vendor.api());
  std::vector<bool> links;
  const Result result = sdk.init(recording_callbacks(&links, nullptr), 60000);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(result.code, kSdkResFailed);
  // The synchronous status during init was delivered (callbacks were
  // installed before the vendor call)...
  ASSERT_EQ(links.size(), 1U);
  // ...but after the failure every state is cleaned: no further callbacks,
  // no cached link state, and SDK calls report NOT_INIT.
  vendor.emit_status(0x01U);
  EXPECT_EQ(links.size(), 1U);
  EXPECT_FALSE(sdk.snapshot().sdk_linked);
  const Result heartbeat = sdk.heartbeat(20);
  EXPECT_FALSE(heartbeat.success());
  EXPECT_EQ(heartbeat.code, kSdkResNotInit);

  // A following init works and installs fresh callbacks.
  vendor.state_->next_init = kSdkResSuccess;
  vendor.state_->init_status = FakeAstrallVendorApi::StatusDelivery::None;
  std::vector<bool> second_links;
  EXPECT_TRUE(sdk.init(recording_callbacks(&second_links, nullptr), 60000)
                  .success());
  vendor.emit_status(0x01U);
  ASSERT_EQ(second_links.size(), 1U);
  EXPECT_TRUE(second_links[0]);
}

TEST(DirectAstrallSdk, SynchronousFirstImuFrameArrivesDuringSubscribe) {
  FakeAstrallVendorApi vendor;
  vendor.state_->subscribe_sync_imu = true;
  vendor.state_->sync_imu = kImuFrame;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::vector<ImuSample> samples;
  const Result result = sdk.subscribe_imu(
      SubscriptionFrequency::Hz125,
      [&samples](const ImuSample& sample) { samples.push_back(sample); }, 20);
  EXPECT_TRUE(result.success());
  // The first frame was delivered synchronously inside the subscribe call.
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].timestamp, 7);
  EXPECT_FLOAT_EQ(samples[0].accelerometer[2], 3.0F);
  EXPECT_FLOAT_EQ(samples[0].gyroscope[0], 0.1F);
  EXPECT_FLOAT_EQ(samples[0].quaternion[3], 1.0F);
  EXPECT_FLOAT_EQ(samples[0].pitch, 0.25F);
  EXPECT_FLOAT_EQ(samples[0].roll, -0.1F);
  EXPECT_FLOAT_EQ(samples[0].yaw, 1.5F);
  EXPECT_FLOAT_EQ(samples[0].odom_x, 0.5F);
  EXPECT_FLOAT_EQ(samples[0].odom_y, -0.5F);
}

TEST(DirectAstrallSdk, SynchronousFirstSportFrameArrivesDuringSubscribe) {
  FakeAstrallVendorApi vendor;
  vendor.state_->subscribe_sync_sport = true;
  vendor.state_->sync_sport = kSportFrame;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::vector<SportSample> samples;
  EXPECT_TRUE(sdk
                  .subscribe_sport(SubscriptionFrequency::Hz50,
                                   [&samples](const SportSample& sample) {
                                     samples.push_back(sample);
                                   },
                                   20)
                  .success());
  ASSERT_EQ(samples.size(), 1U);
  EXPECT_EQ(samples[0].timestamp, 8);
  EXPECT_EQ(samples[0].wheel_speed,
            (std::array<float, 4>{0.1F, 0.2F, 0.3F, 0.4F}));
  // The sport cache backing snapshot().wheel_speed is updated too.
  EXPECT_EQ(sdk.snapshot().wheel_speed, samples[0].wheel_speed);
}

TEST(DirectAstrallSdk, FailedResubscribeRollsBackToPreviousCallback) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  int first = 0;
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&first](const ImuSample&) { ++first; }, 20)
                  .success());

  // Second subscribe fails after delivering a synchronous first frame to
  // the new callback.
  vendor.state_->next_subscribe = kSdkResFailed;
  vendor.state_->subscribe_sync_imu = true;
  vendor.state_->sync_imu = kImuFrame;
  int second = 0;
  const Result result = sdk.subscribe_imu(
      SubscriptionFrequency::Hz25,
      [&second](const ImuSample&) { ++second; }, 20);
  EXPECT_FALSE(result.success());
  EXPECT_EQ(second, 1);  // the synchronously delivered first frame

  // After the rollback, later frames reach the original callback only.
  vendor.emit_imu(kImuFrame);
  EXPECT_EQ(first, 1);
  EXPECT_EQ(second, 1);
}

TEST(DirectAstrallSdk, MapsAuthorityRequestsToVendorTargets) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  vendor.state_->next_authority = kSdkResWithoutAuth;
  const Result denied = sdk.request_authority(true, 20);
  EXPECT_FALSE(denied.success());
  EXPECT_EQ(denied.code, kSdkResWithoutAuth);
  EXPECT_NE(denied.message.find("no control authority"), std::string::npos);

  vendor.state_->next_authority = kSdkResSuccess;
  EXPECT_TRUE(sdk.request_authority(true, 20).success());
  EXPECT_TRUE(sdk.request_authority(false, 20).success());

  const auto calls = vendor.calls();
  std::vector<AuthorityTarget> targets;
  for (const auto& call : calls) {
    if (call.method == "request_authority") {
      targets.push_back(call.authority);
    }
  }
  ASSERT_EQ(targets.size(), 3U);
  EXPECT_EQ(targets[0], AuthorityTarget::Sdk);
  EXPECT_EQ(targets[1], AuthorityTarget::Sdk);
  EXPECT_EQ(targets[2], AuthorityTarget::Joystick);
}

TEST(DirectAstrallSdk, MapsSubscriptionFrequenciesToVendorValues) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  EXPECT_TRUE(sdk.subscribe_imu(SubscriptionFrequency::Hz250, {}, 20).success());
  EXPECT_TRUE(sdk.subscribe_imu(SubscriptionFrequency::Hz25, {}, 20).success());
  EXPECT_TRUE(sdk.subscribe_imu(SubscriptionFrequency::Disabled, {}, 20).success());
  EXPECT_TRUE(sdk.subscribe_sport(SubscriptionFrequency::Hz1, {}, 20).success());

  const auto calls = vendor.calls();
  std::vector<std::uint16_t> frequencies;
  for (const auto& call : calls) {
    if (call.method == "subscribe") {
      frequencies.push_back(call.frequency);
    }
  }
  ASSERT_EQ(frequencies.size(), 4U);
  EXPECT_EQ(frequencies[0], 5U);  // Hz250
  EXPECT_EQ(frequencies[1], 2U);  // Hz25
  EXPECT_EQ(frequencies[2], 0U);  // Disabled
  EXPECT_EQ(frequencies[3], 1U);  // Hz1
}

TEST(DirectAstrallSdk, SnapshotReadsSystemPowerAndSportQueries) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  vendor.state_->system = {2U, 0x00000001U, 0x00000002U};
  vendor.state_->power = {87.5F, 30.0F, 48.5F, 12U, 1U};
  vendor.state_->sport_status = 0xB104U;
  // The sport subscription stores the raw data callback; without it the
  // emit below has nowhere to land and the wheel-speed cache stays empty.
  EXPECT_TRUE(
      sdk.subscribe_sport(SubscriptionFrequency::Hz50, {}, 20).success());
  vendor.emit_sport(kSportFrame);

  const SdkSnapshot snapshot = sdk.snapshot();
  EXPECT_EQ(snapshot.system_status, 2U);
  EXPECT_EQ(snapshot.error_code, 1U);
  EXPECT_EQ(snapshot.warning_code, 2U);
  EXPECT_FLOAT_EQ(snapshot.battery_percentage, 87.5F);
  EXPECT_FLOAT_EQ(snapshot.battery_temperature, 30.0F);
  EXPECT_FLOAT_EQ(snapshot.battery_voltage, 48.5F);
  EXPECT_EQ(snapshot.battery_cycle_count, 12U);
  EXPECT_EQ(snapshot.charge_status, 1U);
  EXPECT_EQ(snapshot.sport_status, 0xB104U);
  EXPECT_EQ(snapshot.wheel_speed,
            (std::array<float, 4>{0.1F, 0.2F, 0.3F, 0.4F}));
}

TEST(DirectAstrallSdk, SnapshotIgnoresSportStatusResultCodes) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
  vendor.state_->sport_status = 0x8004U;  // ASTRALL_RES_FAILED, not a status
  const SdkSnapshot snapshot = sdk.snapshot();
  EXPECT_EQ(snapshot.sport_status, 0U);
}

TEST(DirectAstrallSdk, SnapshotBeforeInitMakesNoVendorCalls) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  const SdkSnapshot snapshot = sdk.snapshot();
  EXPECT_FALSE(snapshot.sdk_linked);
  EXPECT_EQ(snapshot.system_status, 0U);
  EXPECT_FLOAT_EQ(snapshot.battery_percentage, 0.0F);
  EXPECT_EQ(vendor.calls().size(), 0U);
}

TEST(DirectAstrallSdk, DeinitWaitsForInFlightCallbacksAndAllowsReentrancy) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::promise<void> entered;
  std::promise<void> go;
  std::atomic<bool> callback_done{false};
  std::atomic<bool> deinit_done{false};
  Result reentered_heartbeat;
  SdkSnapshot reentered_snapshot;
  bool reentered = false;

  EXPECT_TRUE(sdk
                  .subscribe_imu(
                      SubscriptionFrequency::Hz50,
                      [&](const ImuSample&) {
                        entered.set_value();
                        go.get_future().wait();
                        // Re-enter the SDK while deinit is waiting for us:
                        // this must not deadlock (deinit released the
                        // lifecycle mutex before waiting).
                        reentered = true;
                        reentered_heartbeat = sdk.heartbeat(20);
                        reentered_snapshot = sdk.snapshot();
                        callback_done = true;
                      },
                      20)
                  .success());

  // The fake invokes the stored callback synchronously, so the emission
  // runs on its own thread; otherwise the test thread would block inside
  // the callback waiting for `go` and could never set it.
  std::thread emit_thread([&]() { vendor.emit_imu(kImuFrame); });
  entered.get_future().wait();

  std::thread deinit_thread([&]() {
    sdk.deinit();
    deinit_done = true;
  });
  // Vendor teardown happens before deinit blocks on the gate wait.
  const bool saw_deinit =
      wait_until([&]() { return vendor.deinit_calls() > 0; });
  EXPECT_TRUE(saw_deinit);
  // The project callback is still in flight: deinit must not have returned.
  EXPECT_FALSE(callback_done.load());
  EXPECT_FALSE(deinit_done.load());

  go.set_value();
  deinit_thread.join();
  emit_thread.join();
  EXPECT_TRUE(callback_done.load());
  EXPECT_TRUE(deinit_done.load());
  EXPECT_TRUE(reentered);
  // Re-entering the SDK from a project callback is rejected with a Running
  // failure (never a deadlock, never a vendor call).
  EXPECT_FALSE(reentered_heartbeat.success());
  EXPECT_EQ(reentered_heartbeat.code, kSdkResRunning);
  EXPECT_NE(reentered_heartbeat.message.find("reentrant"),
            std::string::npos);
  // snapshot() re-entry returns only cached state; deinit cleared the cache.
  EXPECT_FALSE(reentered_snapshot.sdk_linked);
}

TEST(DirectAstrallSdk, DeinitCalledFromCallbackIsDeferredAndCompletes) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  bool deinit_returned = false;
  EXPECT_TRUE(sdk
                  .subscribe_imu(
                      SubscriptionFrequency::Hz50,
                      [&](const ImuSample&) {
                        sdk.deinit();  // deferred; must not deadlock
                        deinit_returned = true;
                      },
                      20)
                  .success());
  vendor.emit_imu(kImuFrame);  // synchronous emission
  EXPECT_TRUE(deinit_returned);
  // Only the deferred flag is set; no vendor teardown ran yet.
  EXPECT_EQ(vendor.deinit_calls(), 0);
  EXPECT_FALSE(vendor.deinit_done());

  // A non-callback thread completes the teardown.
  sdk.deinit();
  EXPECT_EQ(vendor.deinit_calls(), 1);
  EXPECT_TRUE(vendor.deinit_done());
  EXPECT_FALSE(sdk.snapshot().sdk_linked);
  // No callbacks are delivered anymore.
  vendor.emit_imu(kImuFrame);
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
}

TEST(DirectAstrallSdk, DefaultConstructionBindsEveryRealVendorFunction) {
  const AstrallVendorApi api = make_real_astrall_vendor_api();
  EXPECT_TRUE(api.init);
  EXPECT_TRUE(api.deinit);
  EXPECT_TRUE(api.heartbeat);
  EXPECT_TRUE(api.request_authority);
  EXPECT_TRUE(api.subscribe);
  EXPECT_TRUE(api.set_mode);
  EXPECT_TRUE(api.move);
  EXPECT_TRUE(api.get_system_status);
  EXPECT_TRUE(api.get_power_status);
  EXPECT_TRUE(api.get_sport_status);
}

TEST(DirectAstrallSdk, NotInitializedCallsNeverTouchVendorBackend) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_FALSE(sdk.heartbeat(20).success());
  EXPECT_FALSE(sdk.request_authority(true, 20).success());
  EXPECT_FALSE(sdk.move({0.0F, 0.0F, 0.0F}, 20).success());
  EXPECT_FALSE(sdk.set_mode(0xA102U, 20).success());
  EXPECT_FALSE(
      sdk.subscribe_imu(SubscriptionFrequency::Hz50, {}, 20).success());
  EXPECT_FALSE(
      sdk.subscribe_sport(SubscriptionFrequency::Hz50, {}, 20).success());
  EXPECT_EQ(vendor.calls().size(), 0U);
}

TEST(DirectAstrallSdk, ReentrantSnapshotFromSyncInitCallbackUsesCacheOnly) {
  FakeAstrallVendorApi vendor;
  vendor.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  vendor.state_->init_status_byte = 0x03U;  // linked + authority
  DirectAstrallSdk sdk(vendor.api());
  SdkSnapshot seen;
  SdkCallbacks callbacks;
  callbacks.on_status = [&](bool /*linked*/, bool /*authority*/) {
    // Re-entry from inside the vendor init call: snapshot() must return
    // only cached state, never call a vendor getter, never deadlock.
    seen = sdk.snapshot();
  };
  EXPECT_TRUE(sdk.init(callbacks, 60000).success());
  EXPECT_TRUE(seen.sdk_linked);
  EXPECT_TRUE(seen.control_authority);
  EXPECT_EQ(vendor.call_count("get_system_status"), 0U);
  EXPECT_EQ(vendor.call_count("get_power_status"), 0U);
  EXPECT_EQ(vendor.call_count("get_sport_status"), 0U);
}

TEST(DirectAstrallSdk, ReentrantCallsFromSyncInitCallbackAreRejected) {
  FakeAstrallVendorApi vendor;
  vendor.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  DirectAstrallSdk sdk(vendor.api());
  Result heartbeat_result;
  Result authority_result;
  SdkCallbacks callbacks;
  callbacks.on_status = [&](bool, bool) {
    heartbeat_result = sdk.heartbeat(20);
    authority_result = sdk.request_authority(true, 20);
  };
  EXPECT_TRUE(sdk.init(callbacks, 60000).success());

  EXPECT_FALSE(heartbeat_result.success());
  EXPECT_EQ(heartbeat_result.code, kSdkResRunning);
  EXPECT_NE(heartbeat_result.message.find("reentrant"), std::string::npos);
  EXPECT_FALSE(authority_result.success());
  EXPECT_EQ(authority_result.code, kSdkResRunning);
  // No vendor entry point may be reached by the re-entrant calls.
  EXPECT_EQ(vendor.call_count("heartbeat"), 0U);
  EXPECT_EQ(vendor.call_count("request_authority"), 0U);
}

TEST(DirectAstrallSdk, DeinitFromSyncInitCallbackIsDeferredUntilVendorReturns) {
  FakeAstrallVendorApi vendor;
  vendor.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  DirectAstrallSdk sdk(vendor.api());
  bool deinit_returned = false;
  SdkCallbacks callbacks;
  callbacks.on_status = [&](bool, bool) {
    // Vendor teardown must NOT run while AstrallSdkInit is still on the
    // call stack; the teardown is deferred to after the init returns.
    sdk.deinit();
    deinit_returned = true;
  };
  const Result init_result = sdk.init(callbacks, 60000);
  EXPECT_TRUE(deinit_returned);  // deinit() itself never blocks
  EXPECT_FALSE(init_result.success());  // init short-circuits after teardown
  EXPECT_NE(init_result.message.find("deinit requested"), std::string::npos);
  EXPECT_EQ(vendor.call_count("init"), 1U);
  EXPECT_EQ(vendor.deinit_calls(), 1);  // teardown completed after init
  // Terminal state: Uninitialized, snapshot unavailable.
  EXPECT_FALSE(sdk.snapshot().sdk_linked);
  // A new init cycle works.
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
}

TEST(DirectAstrallSdk, DeinitOfOtherAdapterFromCallbackWaitsForItsInFlight) {
  FakeAstrallVendorApi vendor_a;
  FakeAstrallVendorApi vendor_b;
  DirectAstrallSdk a(vendor_a.api());
  DirectAstrallSdk b(vendor_b.api());
  EXPECT_TRUE(a.init(SdkCallbacks{}, 60000).success());
  EXPECT_TRUE(b.init(SdkCallbacks{}, 60000).success());

  std::promise<void> b_entered;
  std::promise<void> b_go;
  std::atomic<bool> b_callback_done{false};
  EXPECT_TRUE(b
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) {
                                   b_entered.set_value();
                                   b_go.get_future().wait();
                                   b_callback_done = true;
                                 },
                                 20)
                  .success());
  std::thread b_emit([&]() { vendor_b.emit_imu(kImuFrame); });
  b_entered.get_future().wait();

  // A's callback deinitializes B: this is NOT a re-entrant call into A, so
  // it must take B's normal wait path and block until B's in-flight
  // callback completes.
  std::atomic<bool> a_callback_started{false};
  std::atomic<bool> a_after_deinit{false};
  EXPECT_TRUE(a
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) {
                                   a_callback_started = true;
                                   b.deinit();
                                   a_after_deinit = true;
                                 },
                                 20)
                  .success());
  std::thread a_emit([&]() { vendor_a.emit_imu(kImuFrame); });
  ASSERT_TRUE(wait_until([&]() { return a_callback_started.load(); }));
  // A's callback is blocked inside b.deinit(): B's teardown ran but the
  // wait cannot complete until B's in-flight callback returns.
  EXPECT_FALSE(a_after_deinit.load());
  EXPECT_FALSE(b_callback_done.load());
  EXPECT_EQ(vendor_b.deinit_calls(), 1);

  b_go.set_value();
  b_emit.join();
  ASSERT_TRUE(wait_until([&]() { return a_after_deinit.load(); }));
  a_emit.join();
  EXPECT_TRUE(b_callback_done.load());
  EXPECT_TRUE(a_after_deinit.load());
}

TEST(DirectAstrallSdk, InitRejectedWhileDeinitInProgressThenSucceeds) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::promise<void> entered;
  std::promise<void> go;
  std::atomic<bool> callback_done{false};
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) {
                                   entered.set_value();
                                   go.get_future().wait();
                                   callback_done = true;
                                 },
                                 20)
                  .success());
  std::thread emit([&]() { vendor.emit_imu(kImuFrame); });
  entered.get_future().wait();

  std::atomic<bool> deinit_done{false};
  std::thread deinit([&]() {
    sdk.deinit();
    deinit_done = true;
  });
  // Deterministic: deinit passed vendor teardown and is blocked waiting for
  // the in-flight callback (it has not returned).
  ASSERT_TRUE(wait_until([&]() {
    return vendor.deinit_calls() > 0 && !deinit_done.load();
  }));

  // While ShuttingDown, a new generation must be rejected.
  const Result during = sdk.init(SdkCallbacks{}, 60000);
  EXPECT_FALSE(during.success());
  EXPECT_NE(during.message.find("deinit in progress"), std::string::npos);

  go.set_value();
  emit.join();
  deinit.join();
  EXPECT_TRUE(callback_done.load());
  EXPECT_TRUE(deinit_done.load());

  // After the teardown completes, a fresh init succeeds.
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
  EXPECT_EQ(vendor.call_count("init"), 2U);
}

TEST(DirectAstrallSdk, DeferredTeardownFromCallbackCompletesOnCallerThread) {
  // An async callback requests deinit: only the deferred flag is set, no
  // vendor teardown runs automatically. A later non-callback (runtime)
  // thread completes the teardown inline — on ITS thread, never on the
  // delivery thread.
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::atomic<bool> deinit_returned{false};
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) {
                                   sdk.deinit();  // deferred only
                                   deinit_returned = true;
                                 },
                                 20)
                  .success());

  std::thread emit_thread([&]() { vendor.emit_imu(kImuFrame); });
  const std::thread::id emit_id = emit_thread.get_id();
  emit_thread.join();
  EXPECT_TRUE(deinit_returned.load());
  // No background teardown exists: the deferred flag is kept, nothing ran.
  EXPECT_EQ(vendor.deinit_calls(), 0);
  EXPECT_FALSE(vendor.deinit_done());

  // The non-callback thread completes the teardown inline.
  sdk.deinit();
  EXPECT_EQ(vendor.deinit_calls(), 1);
  EXPECT_TRUE(vendor.deinit_done());
  EXPECT_EQ(vendor.deinit_thread_id(), std::this_thread::get_id());
  EXPECT_NE(vendor.deinit_thread_id(), emit_id);
  EXPECT_FALSE(sdk.snapshot().sdk_linked);
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
}

TEST(DirectAstrallSdk, VendorDeinitWaitsForDeliverWithoutHoldingGate) {
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::promise<void> entered;
  std::promise<void> go;
  std::atomic<bool> callback_done{false};
  std::atomic<bool> emit_finished{false};
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) {
                                   entered.set_value();
                                   go.get_future().wait();
                                   callback_done = true;
                                 },
                                 20)
                  .success());
  std::thread emit_thread([&]() {
    vendor.emit_imu(kImuFrame);
    emit_finished = true;
  });
  entered.get_future().wait();

  // The fake's vendor deinit blocks until the in-flight delivery fully
  // completes (including the gate-protected in-flight decrement). If the
  // adapter held its gate mutex while calling vendor deinit, the delivery
  // could never finish and this test would deadlock.
  vendor.state_->deinit_block_until = [&]() { return emit_finished.load(); };
  std::atomic<bool> deinit_done{false};
  std::thread deinit_thread([&]() {
    sdk.deinit();
    deinit_done = true;
  });

  // deinit must have entered the vendor teardown while the delivery is
  // still blocked.
  ASSERT_TRUE(wait_until([&]() { return vendor.deinit_calls() > 0; }));
  EXPECT_FALSE(callback_done.load());
  EXPECT_FALSE(deinit_done.load());

  go.set_value();
  emit_thread.join();
  deinit_thread.join();
  EXPECT_TRUE(callback_done.load());
  EXPECT_TRUE(emit_finished.load());
  EXPECT_TRUE(deinit_done.load());
  EXPECT_TRUE(vendor.deinit_done());
}

TEST(DirectAstrallSdk, CrossAdapterNestedDeinitCompletesBeforeOuterReturns) {
  FakeAstrallVendorApi vendor_a;
  FakeAstrallVendorApi vendor_b;
  vendor_a.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  vendor_a.state_->init_status_byte = 0x01U;  // linked
  vendor_b.state_->subscribe_sync_imu = true;
  vendor_b.state_->sync_imu = kImuFrame;
  DirectAstrallSdk a(vendor_a.api());
  DirectAstrallSdk b(vendor_b.api());
  EXPECT_TRUE(b.init(SdkCallbacks{}, 60000).success());

  bool b_teardown_completed = false;
  SdkSnapshot b_final_snapshot;
  SdkCallbacks a_callbacks;
  a_callbacks.on_status = [&](bool, bool) {
    // A's synchronous status callback runs while A's vendor call is still
    // on the stack. B's own synchronous subscription callback requests
    // B's deinit; B's teardown must complete before B's subscribe call
    // returns, without waiting for A's outermost checkpoint.
    const Result subscribe_result = b.subscribe_imu(
        SubscriptionFrequency::Hz50,
        [&](const ImuSample&) { b.deinit(); }, 20);
    b_teardown_completed =
        !subscribe_result.success() &&
        subscribe_result.message.find("deinit requested") !=
            std::string::npos;
    b_final_snapshot = b.snapshot();
  };
  EXPECT_TRUE(a.init(a_callbacks, 60000).success());

  EXPECT_TRUE(b_teardown_completed);
  EXPECT_EQ(vendor_b.deinit_calls(), 1);
  EXPECT_EQ(vendor_b.call_count("subscribe"), 1);
  EXPECT_FALSE(b_final_snapshot.sdk_linked);
  // A was unaffected by B's teardown.
  EXPECT_TRUE(a.snapshot().sdk_linked);
  EXPECT_EQ(vendor_a.deinit_calls(), 0);
}

TEST(DirectAstrallSdk, DeinitDuringInFlightCallbackWaitsAndBlocksNewInit) {
  // A non-callback deinit() runs the teardown inline; while an in-flight
  // project callback is still running it waits for in_flight==0. During
  // that wait the ShuttingDown state rejects any new init generation.
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::promise<void> entered;
  std::promise<void> go;
  std::atomic<bool> callback_done{false};
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) {
                                   entered.set_value();
                                   go.get_future().wait();
                                   callback_done = true;
                                 },
                                 20)
                  .success());
  std::thread emit_thread([&]() { vendor.emit_imu(kImuFrame); });
  entered.get_future().wait();

  std::atomic<bool> deinit_done{false};
  std::thread deinit_thread([&]() {
    sdk.deinit();
    deinit_done = true;
  });
  // deinit passed the vendor teardown and is waiting for the in-flight
  // callback (it has not returned yet).
  ASSERT_TRUE(wait_until([&]() {
    return vendor.deinit_calls() > 0 && !deinit_done.load();
  }));

  // New init generations are rejected while ShuttingDown.
  const Result during = sdk.init(SdkCallbacks{}, 60000);
  EXPECT_FALSE(during.success());
  EXPECT_NE(during.message.find("deinit in progress"), std::string::npos);

  go.set_value();
  emit_thread.join();
  deinit_thread.join();
  EXPECT_TRUE(callback_done.load());
  EXPECT_TRUE(deinit_done.load());
  EXPECT_FALSE(sdk.snapshot().sdk_linked);
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
}





TEST(DirectAstrallSdk, AdapterDestroyedInsideCallbackStillRunsVendorTeardown) {
  // Synchronous regression: the adapter's external references are dropped
  // inside a synchronous init status callback right after deinit(); the
  // teardown must still run (checkpoint inline path) and complete.
  FakeAstrallVendorApi vendor;
  vendor.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  std::shared_ptr<DirectAstrallSdk> sdk =
      std::make_shared<DirectAstrallSdk>(vendor.api());
  std::weak_ptr<DirectAstrallSdk> weak_sdk = sdk;
  SdkCallbacks callbacks;
  callbacks.on_status = [&](bool, bool) {
    if (auto s = weak_sdk.lock()) {
      s->deinit();  // deferred; runs at the init tail checkpoint
    }
    // Drop every external reference. A calling reference below keeps the
    // object alive until init returns (destroying an object from inside
    // its own member call would be UB); the drop itself still proves the
    // teardown runs without any surviving external ownership.
    sdk.reset();
  };
  std::shared_ptr<DirectAstrallSdk> calling = sdk;
  const Result init_result = calling->init(callbacks, 60000);
  calling.reset();  // release the calling reference; destructor is idempotent
  EXPECT_FALSE(init_result.success());
  EXPECT_NE(init_result.message.find("deinit requested"), std::string::npos);
  EXPECT_EQ(vendor.deinit_calls(), 1);
  EXPECT_TRUE(vendor.deinit_done());
  EXPECT_TRUE(weak_sdk.expired());
}





TEST(DirectAstrallSdk, AsyncCallbackDeinitWithoutLaterCallerThreadLeavesDeferredFlag) {
  // Contract: a deinit requested inside an async callback only sets the
  // deferred flag. Without a subsequent non-callback thread (deinit() or a
  // vendor entry point whose tail checkpoint picks the flag up), the
  // vendor teardown never runs — the runtime guarantees the follow-up.
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

  std::atomic<bool> deinit_returned{false};
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) {
                                   sdk.deinit();
                                   deinit_returned = true;
                                 },
                                 20)
                  .success());
  std::thread emit_thread([&]() { vendor.emit_imu(kImuFrame); });
  emit_thread.join();
  EXPECT_TRUE(deinit_returned.load());

  // The deferred flag is observable: with no background teardown thread,
  // nothing can have run by the time the delivery returned, so the
  // assertion right after the join is deterministic.
  EXPECT_EQ(vendor.deinit_calls(), 0);
  EXPECT_FALSE(vendor.deinit_done());

  // The runtime follow-up (non-callback thread) completes the teardown.
  sdk.deinit();
  ASSERT_TRUE(wait_until([&]() { return vendor.deinit_done(); }));
  EXPECT_EQ(vendor.deinit_calls(), 1);
  EXPECT_FALSE(sdk.snapshot().sdk_linked);
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
}

TEST(DirectAstrallSdk, ConcurrentDeinitSerializedNoStaleOverwrite) {
  // Regression: two concurrent deinit() calls with the first teardown
  // blocked inside the vendor deinit (fake timing control). The deinit
  // loops are serialized so that every deinit() returns while observing
  // Uninitialized, and a generation initialized afterwards is never
  // stale-overwritten: the vendor deinit count stays consistent with the
  // generation count (1 for the old generation, +1 for an explicit close).
  for (int cycle = 0; cycle < 5; ++cycle) {
    FakeAstrallVendorApi vendor;
    DirectAstrallSdk sdk(vendor.api());
    EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());

    std::atomic<bool> release_a{false};
    vendor.state_->deinit_block_until = [&]() {
      return release_a.load() || vendor.deinit_calls() > 1;
    };

    // Test hook: count how many non-callback threads entered the serialized
    // teardown loop, giving an explicit happens-before edge for the release.
    // The state is captured by value (shared_ptr) so no late event can
    // touch a destroyed local.
    auto entered_count = std::make_shared<std::atomic<int>>(0);
    sdk.set_test_hook([entered_count](TestEvent event) {
      if (event == TestEvent::EnteredEnsureUninitialized) {
        entered_count->fetch_add(1);
      }
    });

    std::atomic<bool> a_done{false};
    std::thread a([&]() {
      sdk.deinit();
      a_done = true;
    });
    ASSERT_TRUE(wait_until([&]() { return vendor.deinit_calls() == 1; }));

    std::atomic<bool> b_done{false};
    std::thread b([&]() {
      sdk.deinit();
      b_done = true;
    });

    // Event-driven release: wait until BOTH callers entered the teardown
    // loop (A is already blocked inside its vendor deinit, B is serialized
    // behind A's teardown_mutex) — a real race, observed via the hook.
    ASSERT_TRUE(wait_until([&]() { return entered_count->load() >= 2; }));
    release_a = true;

    a.join();
    b.join();
    sdk.set_test_hook({});  // clear the hook before the adapter is destroyed
    EXPECT_TRUE(a_done.load());
    EXPECT_TRUE(b_done.load());
    EXPECT_TRUE(vendor.deinit_done());

    // Both deinits observed Uninitialized; a new generation starts cleanly
    // and is not touched by any stale teardown.
    EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
    EXPECT_TRUE(sdk.heartbeat(20).success());
    EXPECT_EQ(vendor.deinit_calls(), 1);
    sdk.deinit();  // explicit close of the new generation
    EXPECT_EQ(vendor.deinit_calls(), 2);
    EXPECT_FALSE(sdk.snapshot().sdk_linked);
  }
}

TEST(DirectAstrallSdk, NestedSyncCallbackReentryIntoOuterAdapterIsRejected) {
  // Cross-adapter nesting: A's synchronous status callback (A's vendor
  // frame on the stack) calls B's subscribe, whose synchronous first frame
  // re-enters A's heartbeat/snapshot. A must treat the re-entry as such
  // (Running failure / cached snapshot) instead of deadlocking on A's
  // lifecycle mutex.
  FakeAstrallVendorApi vendor_a;
  FakeAstrallVendorApi vendor_b;
  vendor_a.state_->init_status = FakeAstrallVendorApi::StatusDelivery::Valid;
  vendor_a.state_->init_status_byte = 0x01U;
  vendor_b.state_->subscribe_sync_imu = true;
  vendor_b.state_->sync_imu = kImuFrame;
  DirectAstrallSdk a(vendor_a.api());
  DirectAstrallSdk b(vendor_b.api());
  EXPECT_TRUE(b.init(SdkCallbacks{}, 60000).success());

  Result reentered_heartbeat;
  SdkSnapshot reentered_snapshot;
  SdkCallbacks a_callbacks;
  a_callbacks.on_status = [&](bool, bool) {
    b.subscribe_imu(SubscriptionFrequency::Hz50,
                    [&](const ImuSample&) {
                      // B's synchronous callback re-enters A while A's
                      // vendor frame is still on the stack.
                      reentered_heartbeat = a.heartbeat(20);
                      reentered_snapshot = a.snapshot();
                    },
                    20);
  };
  EXPECT_TRUE(a.init(a_callbacks, 60000).success());

  EXPECT_FALSE(reentered_heartbeat.success());
  EXPECT_EQ(reentered_heartbeat.code, kSdkResRunning);
  EXPECT_NE(reentered_heartbeat.message.find("reentrant"),
            std::string::npos);
  EXPECT_TRUE(reentered_snapshot.sdk_linked);  // cached status from A's init
  // The re-entry never reached A's vendor backend.
  EXPECT_EQ(vendor_a.call_count("heartbeat"), 0U);
  // After the outer vendor call returns, A works normally.
  EXPECT_TRUE(a.heartbeat(20).success());
  EXPECT_EQ(vendor_a.call_count("heartbeat"), 1U);
}

TEST(DirectAstrallSdk, DeinitWaitsForInFlightCallbackAfterFailedInit) {
  // A failed init leaves an asynchronous status callback in flight (it
  // passed the gate before the failure cleanup). deinit() must wait for
  // in_flight==0 even though the state is already Uninitialized.
  struct Ctx {
    std::mutex mutex;
    RawDataCallback status_cb;
    std::promise<void> gate_passed;
    std::promise<void> release_callback;
    std::atomic<bool> callback_done{false};
    std::thread delivery;
  };
  auto ctx = std::make_shared<Ctx>();
  AstrallVendorApi api;
  api.init = [ctx](RawDataCallback /*heartbeat*/, RawDataCallback status,
                   std::uint32_t) -> std::uint16_t {
    {
      std::lock_guard<std::mutex> lock(ctx->mutex);
      ctx->status_cb = std::move(status);
    }
    // Vendor behaviour: an async delivery is already started when the init
    // failure is returned.
    RawDataCallback cb;
    {
      std::lock_guard<std::mutex> lock(ctx->mutex);
      cb = ctx->status_cb;
    }
    ctx->delivery = std::thread([ctx, cb]() {
      std::uint8_t byte = 0x01U;
      cb(&byte, 1);  // passes the gate (Initializing), then blocks
    });
    // Return the failure only after the callback passed the gate.
    ctx->gate_passed.get_future().wait();
    return kSdkResFailed;
  };
  api.deinit = []() {};
  api.heartbeat = [](std::uint32_t) { return kSdkResFailed; };
  api.request_authority = [](AuthorityTarget, std::uint32_t) {
    return kSdkResFailed;
  };
  api.subscribe = [](SubscriptionTopic, std::uint16_t, RawDataCallback,
                     std::uint32_t) { return kSdkResFailed; };
  api.set_mode = [](std::uint16_t, std::uint32_t) { return kSdkResFailed; };
  api.move = [](float, float, float, std::uint32_t) { return kSdkResFailed; };
  api.get_system_status = [](RawSystemStatus&) { return kSdkResFailed; };
  api.get_power_status = [](RawPowerStatus&) { return kSdkResFailed; };
  api.get_sport_status = []() { return kSdkResFailed; };

  DirectAstrallSdk sdk(api);
  SdkCallbacks callbacks;
  callbacks.on_status = [ctx](bool, bool) {
    ctx->gate_passed.set_value();  // callback is now in flight (in_flight>0)
    ctx->release_callback.get_future().wait();
    ctx->callback_done = true;
  };
  EXPECT_FALSE(sdk.init(callbacks, 60000).success());  // failed init
  EXPECT_FALSE(ctx->callback_done.load());  // async callback still blocked

  // Test hook: ShuttingDownPublished gives an explicit happens-before edge
  // for the assertions below. The state is captured by value (shared_ptr).
  auto shutting_down = std::make_shared<std::atomic<bool>>(false);
  sdk.set_test_hook([shutting_down](TestEvent event) {
    if (event == TestEvent::ShuttingDownPublished) {
      *shutting_down = true;
    }
  });
  std::atomic<bool> deinit_done{false};
  std::thread deinit_thread([&]() {
    sdk.deinit();
    deinit_done = true;
  });
  // deinit must NOT return while the in-flight callback is still running;
  // ShuttingDownPublished proves deinit already closed the gate, and the
  // callback is still blocked, so this assertion is deterministic.
  ASSERT_TRUE(wait_until([&]() { return shutting_down->load(); }));
  EXPECT_FALSE(deinit_done.load());

  ctx->release_callback.set_value();
  deinit_thread.join();
  ctx->delivery.join();
  sdk.set_test_hook({});
  EXPECT_TRUE(ctx->callback_done.load());
  EXPECT_TRUE(deinit_done.load());
}

TEST(DirectAstrallSdk, VendorEntryTailCheckpointConsumesDeferredTeardown) {
  // A deferred teardown requested from an async callback is completed by
  // the tail checkpoint of a later ordinary vendor entry point — no deinit
  // involved.
  FakeAstrallVendorApi vendor;
  DirectAstrallSdk sdk(vendor.api());
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample&) { sdk.deinit(); },
                                 20)
                  .success());
  vendor.emit_imu(kImuFrame);  // synchronous delivery -> deferred flag
  EXPECT_EQ(vendor.deinit_calls(), 0);
  EXPECT_FALSE(vendor.deinit_done());

  // A plain heartbeat's tail checkpoint picks the flag up and completes
  // the teardown inline on this (non-callback) thread.
  const Result r = sdk.heartbeat(20);
  EXPECT_FALSE(r.success());
  EXPECT_NE(r.message.find("deinit requested"), std::string::npos);
  EXPECT_EQ(vendor.deinit_calls(), 1);
  EXPECT_TRUE(vendor.deinit_done());
  EXPECT_FALSE(sdk.snapshot().sdk_linked);
  EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
}

TEST(DirectAstrallSdk, TailCheckpointTeardownConcurrentWithDeinitSerialized) {
  // Regression: a tail checkpoint (heartbeat after a deferred teardown)
  // and a concurrent deinit() both complete the teardown; serialization
  // guarantees a single publisher, consistent final state, and a clean new
  // generation afterwards.
  for (int cycle = 0; cycle < 5; ++cycle) {
    FakeAstrallVendorApi vendor;
    DirectAstrallSdk sdk(vendor.api());
    EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
    EXPECT_TRUE(sdk
                    .subscribe_imu(SubscriptionFrequency::Hz50,
                                   [&](const ImuSample&) { sdk.deinit(); },
                                   20)
                    .success());
    vendor.emit_imu(kImuFrame);  // synchronous delivery -> deferred flag
    EXPECT_EQ(vendor.deinit_calls(), 0);

    std::atomic<bool> release_vd{false};
    vendor.state_->deinit_block_until = [&]() {
      return release_vd.load() || vendor.deinit_calls() > 1;
    };

    // Test hook: wait until BOTH the checkpoint thread and the deinit
    // thread entered the serialized teardown loop (T1 blocked inside its
    // vendor deinit, T2 behind T1's teardown_mutex) before releasing.
    auto entered_count = std::make_shared<std::atomic<int>>(0);
    sdk.set_test_hook([entered_count](TestEvent event) {
      if (event == TestEvent::EnteredEnsureUninitialized) {
        entered_count->fetch_add(1);
      }
    });
    std::thread t1([&]() { sdk.heartbeat(20); });
    ASSERT_TRUE(wait_until([&]() { return vendor.deinit_calls() == 1; }));
    std::thread t2([&]() { sdk.deinit(); });
    ASSERT_TRUE(wait_until([&]() { return entered_count->load() >= 2; }));
    release_vd = true;
    t1.join();
    t2.join();
    sdk.set_test_hook({});
    EXPECT_TRUE(vendor.deinit_done());

    // Consistent: exactly one vendor deinit for the old generation, a new
    // generation starts cleanly and is not touched by any stale teardown.
    EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
    EXPECT_TRUE(sdk.heartbeat(20).success());
    EXPECT_EQ(vendor.deinit_calls(), 1);
    sdk.deinit();  // explicit close of the new generation
    EXPECT_EQ(vendor.deinit_calls(), 2);
  }
}

TEST(DirectAstrallSdk, FailedInitDrainBlocksNewInitUntilInFlightClears) {
  // During the failed-init drain (state already Uninitialized but an async
  // callback still in flight) the adapter must move to ShuttingDown: a new
  // init attempt is rejected ("deinit in progress") instead of starting a
  // generation that the drain's publication would stale-overwrite.
  struct Ctx {
    std::mutex mutex;
    RawDataCallback status_cb;
    std::promise<void> gate_passed;
    std::promise<void> release_callback;
    std::atomic<bool> callback_done{false};
    std::atomic<int> init_calls{0};
    std::thread delivery;
  };
  auto ctx = std::make_shared<Ctx>();
  AstrallVendorApi api;
  api.init = [ctx](RawDataCallback /*heartbeat*/, RawDataCallback status,
                   std::uint32_t) -> std::uint16_t {
    if (ctx->init_calls.fetch_add(1) > 0) {
      // Later init attempts succeed without any delivery (so a rejection
      // during the drain is distinguishable from a vendor failure).
      return kSdkResSuccess;
    }
    {
      std::lock_guard<std::mutex> lock(ctx->mutex);
      ctx->status_cb = std::move(status);
    }
    RawDataCallback cb;
    {
      std::lock_guard<std::mutex> lock(ctx->mutex);
      cb = ctx->status_cb;
    }
    ctx->delivery = std::thread([ctx, cb]() {
      std::uint8_t byte = 0x01U;
      cb(&byte, 1);  // passes the gate (Initializing), then blocks
    });
    ctx->gate_passed.get_future().wait();
    return kSdkResFailed;
  };
  api.deinit = []() {};
  api.heartbeat = [](std::uint32_t) { return kSdkResFailed; };
  api.request_authority = [](AuthorityTarget, std::uint32_t) {
    return kSdkResFailed;
  };
  api.subscribe = [](SubscriptionTopic, std::uint16_t, RawDataCallback,
                     std::uint32_t) { return kSdkResFailed; };
  api.set_mode = [](std::uint16_t, std::uint32_t) { return kSdkResFailed; };
  api.move = [](float, float, float, std::uint32_t) { return kSdkResFailed; };
  api.get_system_status = [](RawSystemStatus&) { return kSdkResFailed; };
  api.get_power_status = [](RawPowerStatus&) { return kSdkResFailed; };
  api.get_sport_status = []() { return kSdkResFailed; };

  // Lifecycle-safe hook state captured BY VALUE (shared_ptr): even an
  // unexpected late event can never touch a destroyed local.
  auto shutting_down = std::make_shared<std::atomic<bool>>(false);
  auto hook_calls = std::make_shared<std::atomic<int>>(0);
  int before_clear = 0;
  {
    DirectAstrallSdk sdk(api);
    // Test hook installed BEFORE the deinit thread starts, so the
    // ShuttingDownPublished event can never be missed.
    sdk.set_test_hook([shutting_down, hook_calls](TestEvent event) {
      hook_calls->fetch_add(1);
      if (event == TestEvent::ShuttingDownPublished) {
        *shutting_down = true;
      }
    });

    SdkCallbacks callbacks;
    callbacks.on_status = [ctx](bool, bool) {
      ctx->gate_passed.set_value();  // callback in flight (in_flight>0)
      ctx->release_callback.get_future().wait();
      ctx->callback_done = true;
    };
    EXPECT_FALSE(sdk.init(callbacks, 60000).success());  // failed init
    EXPECT_FALSE(ctx->callback_done.load());

    std::atomic<bool> deinit_done{false};
    std::thread deinit_thread([&]() {
      sdk.deinit();
      deinit_done = true;
    });
    ASSERT_TRUE(wait_until([&]() { return shutting_down->load(); }));
    // The drain is waiting for the in-flight callback (blocked), so deinit
    // cannot have returned yet.
    EXPECT_FALSE(deinit_done.load());

    // During the drain a new init must be rejected (deterministic: the
    // ShuttingDown state was already published).
    const Result mid = sdk.init(SdkCallbacks{}, 60000);
    EXPECT_FALSE(mid.success());
    EXPECT_NE(mid.message.find("deinit in progress"), std::string::npos);
    if (mid.success()) {
      ctx->release_callback.set_value();
      deinit_thread.join();
      ctx->delivery.join();
      FAIL() << "init succeeded while the failed-init drain was waiting";
      return;
    }

    ctx->release_callback.set_value();
    deinit_thread.join();
    ctx->delivery.join();
    EXPECT_TRUE(ctx->callback_done.load());
    EXPECT_TRUE(deinit_done.load());

    // After the drain completes, a new init succeeds (it is no longer
    // rejected as "in progress" and nothing stale-overwrites it).
    EXPECT_TRUE(sdk.init(SdkCallbacks{}, 60000).success());
    // Explicitly clear the hook: destroying the adapter below
    // (state==Initialized -> teardown) must NOT invoke it any more.
    sdk.set_test_hook({});
    before_clear = hook_calls->load();
  }  // sdk destroyed here (teardown runs; hook already cleared)
  EXPECT_EQ(hook_calls->load(), before_clear);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
