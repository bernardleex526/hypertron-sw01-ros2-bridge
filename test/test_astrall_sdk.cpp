#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fake_astrall_sdk.hpp"
#include "hypertron_ros2_bridge/astrall_sdk.hpp"

namespace hypertron_ros2_bridge {
namespace {

using test::FakeAstrallSdk;

TEST(SdkResult, SuccessAndFailureCodesAreReadable) {
  const Result ok = Result::ok();
  EXPECT_TRUE(ok.success());
  EXPECT_EQ(ok.code, kSdkResSuccess);

  const Result failure = Result::failure(kSdkResTimeout, "vendor timeout");
  EXPECT_FALSE(failure.success());
  EXPECT_EQ(failure.message, "vendor timeout");
}

TEST(SdkResult, DescribeAllVendorResultCodes) {
  EXPECT_EQ(describe_sdk_result_code(kSdkResFailed),
            "execution or request failed");
  EXPECT_EQ(describe_sdk_result_code(kSdkResTimeout), "timeout");
  EXPECT_EQ(describe_sdk_result_code(kSdkResRunning),
            "operation still running");
  EXPECT_EQ(describe_sdk_result_code(kSdkResSuccess), "success");
  EXPECT_EQ(describe_sdk_result_code(kSdkResInvalidParam), "invalid parameter");
  EXPECT_EQ(describe_sdk_result_code(kSdkResNotInit), "SDK not initialized");
  EXPECT_EQ(describe_sdk_result_code(kSdkResRcNoRelease),
            "remote controller has not released authority");
  EXPECT_EQ(describe_sdk_result_code(kSdkResBeenObtained),
            "control authority already held by another device");
  EXPECT_EQ(describe_sdk_result_code(kSdkResWithoutAuth), "no control authority");
  const std::string unknown = describe_sdk_result_code(0x1234U);
  EXPECT_NE(unknown.find("0x1234"), std::string::npos);
}

TEST(SubscriptionFrequency, ValuesMatchDocumentedVendorOrdering) {
  EXPECT_EQ(static_cast<std::uint16_t>(SubscriptionFrequency::Disabled), 0U);
  EXPECT_EQ(static_cast<std::uint16_t>(SubscriptionFrequency::Hz1), 1U);
  EXPECT_EQ(static_cast<std::uint16_t>(SubscriptionFrequency::Hz25), 2U);
  EXPECT_EQ(static_cast<std::uint16_t>(SubscriptionFrequency::Hz50), 3U);
  EXPECT_EQ(static_cast<std::uint16_t>(SubscriptionFrequency::Hz125), 4U);
  EXPECT_EQ(static_cast<std::uint16_t>(SubscriptionFrequency::Hz250), 5U);
}

TEST(FakeAstrallSdk, RecordsCallsAndReturnsProgrammableResults) {
  FakeAstrallSdk sdk;
  SdkCallbacks callbacks;

  sdk.next_init = Result::failure(kSdkResFailed, "programmed init failure");
  EXPECT_FALSE(sdk.init(callbacks, 60000).success());

  sdk.next_init = Result::ok();
  EXPECT_TRUE(sdk.init(callbacks, 60000).success());

  sdk.next_heartbeat = Result::failure(kSdkResTimeout, "programmed timeout");
  const Result heartbeat = sdk.heartbeat(20);
  EXPECT_FALSE(heartbeat.success());
  EXPECT_EQ(heartbeat.code, kSdkResTimeout);

  sdk.next_authority = Result::failure(kSdkResBeenObtained, "blocked");
  EXPECT_FALSE(sdk.request_authority(true, 20).success());
  sdk.next_authority = Result::ok();
  EXPECT_TRUE(sdk.request_authority(false, 20).success());

  sdk.next_subscribe_imu = Result::failure(kSdkResInvalidParam, "bad freq");
  EXPECT_FALSE(
      sdk.subscribe_imu(SubscriptionFrequency::Hz250, {}, 20).success());
  sdk.next_subscribe_imu = Result::ok();
  EXPECT_TRUE(
      sdk.subscribe_imu(SubscriptionFrequency::Hz125, {}, 20).success());

  sdk.next_subscribe_sport = Result::ok();
  EXPECT_TRUE(
      sdk.subscribe_sport(SubscriptionFrequency::Hz50, {}, 20).success());

  sdk.next_move = Result::failure(kSdkResWithoutAuth, "no authority");
  EXPECT_FALSE(sdk.move({0.1F, -0.2F, 0.3F}, 20).success());
  sdk.next_move = Result::ok();
  EXPECT_TRUE(sdk.move({0.1F, 0.0F, 0.0F}, 20).success());

  sdk.next_mode = Result::ok();
  EXPECT_TRUE(sdk.set_mode(0xA102U, 20).success());

  EXPECT_EQ(sdk.call_count("init"), 2U);
  EXPECT_EQ(sdk.call_count("heartbeat"), 1U);
  EXPECT_EQ(sdk.call_count("request_authority"), 2U);
  EXPECT_EQ(sdk.call_count("subscribe_imu"), 2U);
  EXPECT_EQ(sdk.call_count("subscribe_sport"), 1U);
  EXPECT_EQ(sdk.call_count("move"), 2U);
  EXPECT_EQ(sdk.call_count("set_mode"), 1U);
  EXPECT_EQ(sdk.call_count("deinit"), 0U);

  const auto calls = sdk.calls();
  ASSERT_EQ(calls.size(), 11U);

  const auto& authority_ok = calls[4];
  EXPECT_EQ(authority_ok.method, "request_authority");
  EXPECT_FALSE(authority_ok.authority_sdk);  // false -> joystick authority
  EXPECT_EQ(authority_ok.timeout_ms, 20U);

  const auto& imu_subscribe = calls[6];
  EXPECT_EQ(imu_subscribe.method, "subscribe_imu");
  EXPECT_EQ(imu_subscribe.frequency, SubscriptionFrequency::Hz125);
  EXPECT_EQ(imu_subscribe.timeout_ms, 20U);

  const auto& move_call = calls[9];
  EXPECT_EQ(move_call.method, "move");
  EXPECT_FLOAT_EQ(move_call.velocity.vx, 0.1F);
  EXPECT_FLOAT_EQ(move_call.velocity.vy, 0.0F);

  const auto& mode_call = calls[10];
  EXPECT_EQ(mode_call.method, "set_mode");
  EXPECT_EQ(mode_call.mode, 0xA102U);
}

TEST(FakeAstrallSdk, TriggersStoredCallbacksWithCopiedSamples) {
  FakeAstrallSdk sdk;
  SdkCallbacks callbacks;
  int status_count = 0;
  bool saw_linked = false;
  bool saw_authority = false;
  callbacks.on_status = [&](bool linked, bool authority) {
    ++status_count;
    saw_linked = linked;
    saw_authority = authority;
  };
  EXPECT_TRUE(sdk.init(callbacks, 60000).success());

  std::vector<ImuSample> imu_seen;
  std::vector<SportSample> sport_seen;
  EXPECT_TRUE(sdk
                  .subscribe_imu(SubscriptionFrequency::Hz50,
                                 [&](const ImuSample& sample) {
                                   imu_seen.push_back(sample);
                                 },
                                 20)
                  .success());
  EXPECT_TRUE(sdk
                  .subscribe_sport(SubscriptionFrequency::Hz25,
                                   [&](const SportSample& sample) {
                                     sport_seen.push_back(sample);
                                   },
                                   20)
                  .success());

  sdk.emit_status(true, true);
  sdk.emit_status(false, false);

  ImuSample imu;
  imu.timestamp = 42;
  imu.accelerometer = {1.0F, 2.0F, 3.0F};
  imu.gyroscope = {0.1F, 0.2F, 0.3F};
  imu.quaternion = {0.0F, 0.0F, 0.0F, 1.0F};
  imu.pitch = 0.25F;
  imu.roll = -0.1F;
  imu.yaw = 1.5F;
  imu.odom_x = 0.5F;
  imu.odom_y = -0.5F;
  sdk.emit_imu(imu);

  SportSample sport;
  sport.timestamp = 43;
  sport.wheel_speed = {0.1F, 0.2F, 0.3F, 0.4F};
  sdk.emit_sport(sport);

  EXPECT_EQ(status_count, 2);
  EXPECT_FALSE(saw_linked);
  EXPECT_FALSE(saw_authority);  // values from the last emission
  ASSERT_EQ(imu_seen.size(), 1U);
  EXPECT_EQ(imu_seen[0].timestamp, 42);
  EXPECT_FLOAT_EQ(imu_seen[0].accelerometer[2], 3.0F);
  EXPECT_FLOAT_EQ(imu_seen[0].yaw, 1.5F);
  EXPECT_FLOAT_EQ(imu_seen[0].odom_y, -0.5F);
  ASSERT_EQ(sport_seen.size(), 1U);
  EXPECT_EQ(sport_seen[0].timestamp, 43);
  EXPECT_EQ(sport_seen[0].wheel_speed, sport.wheel_speed);
  // Sport emission updates the snapshot wheel speeds.
  EXPECT_EQ(sdk.snapshot().wheel_speed, sport.wheel_speed);

  sdk.deinit();
  EXPECT_EQ(sdk.call_count("deinit"), 1U);
  sdk.deinit();  // repeated deinit is recorded and safe
  EXPECT_EQ(sdk.call_count("deinit"), 2U);
  // Deinit clears the stored callbacks.
  sdk.emit_status(true, true);
  EXPECT_EQ(status_count, 2);
  sdk.emit_imu(imu);
  EXPECT_EQ(imu_seen.size(), 1U);
}

TEST(FakeAstrallSdk, SnapshotIsProgrammable) {
  FakeAstrallSdk sdk;
  SdkSnapshot snapshot;
  snapshot.sdk_linked = true;
  snapshot.control_authority = true;
  snapshot.system_status = 2U;
  snapshot.battery_percentage = 87.5F;
  snapshot.wheel_speed = {1.0F, 2.0F, 3.0F, 4.0F};
  sdk.snapshot_value = snapshot;
  const SdkSnapshot seen = sdk.snapshot();
  EXPECT_TRUE(seen.sdk_linked);
  EXPECT_TRUE(seen.control_authority);
  EXPECT_EQ(seen.system_status, 2U);
  EXPECT_FLOAT_EQ(seen.battery_percentage, 87.5F);
  EXPECT_EQ(seen.wheel_speed, snapshot.wheel_speed);
}

TEST(FakeAstrallSdk, SuccessfulAuthorityRequestUpdatesSnapshotAuthority) {
  FakeAstrallSdk sdk;
  SdkCallbacks callbacks;
  EXPECT_TRUE(sdk.init(callbacks, 60000).success());
  sdk.next_authority = Result::ok();
  EXPECT_TRUE(sdk.request_authority(true, 20).success());
  EXPECT_TRUE(sdk.snapshot().control_authority);
  EXPECT_TRUE(sdk.request_authority(false, 20).success());
  EXPECT_FALSE(sdk.snapshot().control_authority);
}

TEST(DirectAstrallSdk, RejectsAllCallsBeforeInitWithoutTouchingVendorSdk) {
  // Constructing the adapter must not call into the vendor library, so unit
  // tests run with no robot and no SDK side effects.
  DirectAstrallSdk sdk;

  const Result heartbeat = sdk.heartbeat(20);
  EXPECT_FALSE(heartbeat.success());
  EXPECT_EQ(heartbeat.code, kSdkResNotInit);
  EXPECT_EQ(heartbeat.message, "SDK not initialized");

  EXPECT_FALSE(sdk.request_authority(true, 20).success());
  EXPECT_FALSE(sdk.move({0.0F, 0.0F, 0.0F}, 20).success());
  EXPECT_FALSE(sdk.set_mode(0xA102U, 20).success());
  EXPECT_FALSE(
      sdk.subscribe_imu(SubscriptionFrequency::Hz50, {}, 20).success());
  EXPECT_FALSE(
      sdk.subscribe_sport(SubscriptionFrequency::Hz50, {}, 20).success());

  const SdkSnapshot snapshot = sdk.snapshot();
  EXPECT_FALSE(snapshot.sdk_linked);
  EXPECT_FALSE(snapshot.control_authority);
  EXPECT_EQ(snapshot.system_status, 0U);
  EXPECT_EQ(snapshot.error_code, 0U);
  EXPECT_FLOAT_EQ(snapshot.battery_percentage, 0.0F);

  sdk.deinit();  // idempotent and safe before init
}

}  // namespace
}  // namespace hypertron_ros2_bridge
