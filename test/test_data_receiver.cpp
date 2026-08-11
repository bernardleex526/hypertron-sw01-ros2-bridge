#include <array>
#include <chrono>
#include <cmath>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/data_receiver.hpp"

namespace hypertron_ros2_bridge {
namespace {

DataReceiverConfig config() {
  DataReceiverConfig value;
  value.imu_frame = "imu_link";
  value.odom_frame = "odom";
  value.base_frame = "base_link";
  value.timestamp_source = TimestampSource::Receive;
  value.quaternion_order = QuaternionOrder::Xyzw;
  return value;
}

TEST(DataReceiver, MapsImuXyzwFramesAndCovariance) {
  ImuPayload payload;
  payload.device_time = 999;
  payload.acceleration = {1, 2, 3};
  payload.angular_velocity = {4, 5, 6};
  payload.quaternion = {0, 0, 0, 2};
  auto cfg = config();
  cfg.orientation_covariance_diagonal = {0.1, 0.2, 0.3};
  const auto sample = to_imu_sample(payload, 123456U, cfg);
  EXPECT_EQ(sample.frame_id, "imu_link");
  EXPECT_EQ(sample.timestamp_ns, 123456U);
  EXPECT_DOUBLE_EQ(sample.linear_acceleration[0], 1.0);
  EXPECT_DOUBLE_EQ(sample.orientation[3], 1.0);
  EXPECT_DOUBLE_EQ(sample.orientation_covariance[0], 0.1);
  EXPECT_DOUBLE_EQ(sample.orientation_covariance[4], 0.2);
  EXPECT_DOUBLE_EQ(sample.orientation_covariance[8], 0.3);
}

TEST(DataReceiver, SupportsWxyzAndDeviceTimestampUnits) {
  ImuPayload payload;
  payload.device_time = 12;
  payload.quaternion = {1, 0, 0, 0};
  auto cfg = config();
  cfg.quaternion_order = QuaternionOrder::Wxyz;
  cfg.timestamp_source = TimestampSource::DeviceMilliseconds;
  const auto sample = to_imu_sample(payload, 1, cfg);
  EXPECT_EQ(sample.timestamp_ns, 12000000U);
  EXPECT_DOUBLE_EQ(sample.orientation[3], 1.0);
}

TEST(DataReceiver, RejectsZeroQuaternionAndNonFiniteValues) {
  ImuPayload payload;
  EXPECT_THROW(to_imu_sample(payload, 1, config()), MappingError);
  payload.quaternion = {0, 0, 0, 1};
  payload.acceleration[0] = NAN;
  EXPECT_THROW(to_imu_sample(payload, 1, config()), MappingError);
}

TEST(DataReceiver, MapsOdometryFramesAndNormalizesQuaternion) {
  OdometryPayload payload;
  payload.device_time = 20;
  payload.position = {1, -2, 0.5};
  payload.orientation = {0, 0, 0, 4};
  const auto sample = to_odometry_sample(payload, 300, config());
  EXPECT_EQ(sample.frame_id, "odom");
  EXPECT_EQ(sample.child_frame_id, "base_link");
  EXPECT_EQ(sample.position, payload.position);
  EXPECT_DOUBLE_EQ(sample.orientation[3], 1.0);
}

TEST(DataReceiver, CameraDisabledIsNonFatalAndCountsDrop) {
  CameraIngestState state(false);
  EXPECT_FALSE(state.accept({1, 1, 1, false, {0, 0, 1}}));
  EXPECT_EQ(state.disabled_drops(), 1U);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
