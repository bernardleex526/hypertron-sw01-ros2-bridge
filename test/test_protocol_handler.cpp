#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/protocol_handler.hpp"

namespace hypertron_ros2_bridge {
namespace {

Frame ping(std::uint32_t sequence) {
  return {MessageType::Ping, sequence, 1000U + sequence, {}};
}

void put_u16_le(std::vector<std::uint8_t>& bytes, std::size_t offset,
                std::uint16_t value) {
  bytes[offset] = static_cast<std::uint8_t>(value);
  bytes[offset + 1] = static_cast<std::uint8_t>(value >> 8U);
}

void put_u32_le(std::vector<std::uint8_t>& bytes, std::size_t offset,
                std::uint32_t value) {
  for (std::size_t i = 0; i < 4; ++i) {
    bytes[offset + i] = static_cast<std::uint8_t>(value >> (i * 8U));
  }
}

void put_i64_le(std::vector<std::uint8_t>& bytes, std::size_t offset,
                std::int64_t value) {
  const auto bits = static_cast<std::uint64_t>(value);
  for (std::size_t i = 0; i < 8; ++i) {
    bytes[offset + i] = static_cast<std::uint8_t>(bits >> (i * 8U));
  }
}

std::vector<std::uint8_t> packed_odometry_fixture() {
  std::vector<std::uint8_t> bytes(68, 0);
  put_u16_le(bytes, 0, 0xAA55U);
  put_i64_le(bytes, 2, 99);
  put_i64_le(bytes, 10, 1000000);
  put_i64_le(bytes, 18, -2000000);
  put_i64_le(bytes, 26, 500000);
  put_i64_le(bytes, 34, 0);
  put_i64_le(bytes, 42, 0);
  put_i64_le(bytes, 50, 0);
  put_i64_le(bytes, 58, 1000000);
  put_u16_le(bytes, 66, 0xFF00U);
  return bytes;
}

std::vector<std::uint8_t> aligned_odometry_fixture() {
  std::vector<std::uint8_t> bytes(80, 0);
  put_u16_le(bytes, 0, 0xAA55U);
  put_i64_le(bytes, 8, 100);
  put_i64_le(bytes, 16, 1);
  put_i64_le(bytes, 24, 2);
  put_i64_le(bytes, 32, 3);
  put_i64_le(bytes, 40, 0);
  put_i64_le(bytes, 48, 0);
  put_i64_le(bytes, 56, 0);
  put_i64_le(bytes, 64, 1);
  put_u16_le(bytes, 72, 0xFF00U);
  return bytes;
}

std::vector<std::uint8_t> point_cloud_fixture(std::uint16_t count) {
  std::vector<std::uint8_t> bytes(20U + 28U * count + 2U, 0);
  put_u16_le(bytes, 0, 0xAA55U);
  put_i64_le(bytes, 2, 200);
  put_u32_le(bytes, 10, 7);
  put_u32_le(bytes, 14, 8);
  put_u16_le(bytes, 18, count);
  if (count > 0U) {
    put_u32_le(bytes, 20, static_cast<std::uint32_t>(1000));
    put_u32_le(bytes, 24, static_cast<std::uint32_t>(-2000));
    put_u32_le(bytes, 28, static_cast<std::uint32_t>(500));
    put_u32_le(bytes, 32, 0x11223344U);
    put_u32_le(bytes, 36, 0x55667788U);
    put_u32_le(bytes, 40, 0x99AABBCCU);
    put_u32_le(bytes, 44, 0xDDEEFF00U);
  }
  put_u16_le(bytes, bytes.size() - 2U, 0xFF00U);
  return bytes;
}

TEST(ProtocolHandler, RoundTripsNetworkOrderFrame) {
  Frame input{MessageType::CmdVelocity, 42, 123456789,
              encode_velocity({0.5F, -0.25F, 1.0F})};
  const auto bytes = ProtocolHandler::encode(input);
  ASSERT_EQ(bytes.size(), 40U);
  EXPECT_EQ(bytes[0], 'H');
  EXPECT_EQ(bytes[1], 'T');
  EXPECT_EQ(bytes[2], 'B');
  EXPECT_EQ(bytes[3], 'R');
  EXPECT_EQ(bytes[4], 1U);
  EXPECT_EQ(bytes[5], static_cast<std::uint8_t>(MessageType::CmdVelocity));
  EXPECT_EQ(bytes[8], 0U);
  EXPECT_EQ(bytes[11], 42U);
  ProtocolHandler decoder;
  const auto frames = decoder.feed(bytes);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front(), input);
}

TEST(ProtocolHandler, BuffersHalfFrameAndSeparatesStickyFrames) {
  const auto a = ProtocolHandler::encode(ping(1));
  const auto b = ProtocolHandler::encode(ping(2));
  ProtocolHandler decoder;
  const std::vector<std::uint8_t> first(a.begin(), a.begin() + 7);
  EXPECT_TRUE(decoder.feed(first).empty());
  std::vector<std::uint8_t> rest(a.begin() + 7, a.end());
  rest.insert(rest.end(), b.begin(), b.end());
  const auto frames = decoder.feed(rest);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0], ping(1));
  EXPECT_EQ(frames[1], ping(2));
}

TEST(ProtocolHandler, RejectsBadCrcAndOversize) {
  auto bytes = ProtocolHandler::encode(ping(7));
  bytes.back() ^= 0x80U;
  ProtocolHandler decoder(1024);
  EXPECT_THROW(decoder.feed(bytes), ProtocolError);

  auto velocity = ProtocolHandler::encode(
      {MessageType::CmdVelocity, 1, 1, encode_velocity({0, 0, 0})});
  velocity[12] = 0;
  velocity[13] = 0;
  velocity[14] = 4;
  velocity[15] = 1;
  EXPECT_THROW(decoder.feed(velocity), ProtocolError);
}

TEST(ProtocolHandler, RejectsUnknownVersionAndFlags) {
  auto bad_version = ProtocolHandler::encode(ping(1));
  bad_version[4] = 2;
  ProtocolHandler decoder;
  EXPECT_THROW(decoder.feed(bad_version), ProtocolError);

  auto bad_flags = ProtocolHandler::encode(ping(2));
  bad_flags[7] = 1;
  EXPECT_THROW(decoder.feed(bad_flags), ProtocolError);
}

TEST(ProtocolHandler, EmptyNullFeedIsANoOp) {
  ProtocolHandler decoder;
  EXPECT_TRUE(decoder.feed(nullptr, 0).empty());
}

TEST(PayloadCodec, RoundTripsHandshakeControlAndResponses) {
  const HelloPayload hello{1, 1, 0x25U, 0x01020304U};
  EXPECT_EQ(decode_hello(encode_hello(hello)), hello);
  const HelloAckPayload hello_ack{1, 0x25U, 99U, "ASTRALL-1.0.7"};
  EXPECT_EQ(decode_hello_ack(encode_hello_ack(hello_ack)), hello_ack);
  const VelocityPayload velocity{0.5F, -0.25F, 1.0F};
  const auto velocity_bytes = encode_velocity(velocity);
  EXPECT_EQ(velocity_bytes,
            (std::vector<std::uint8_t>{0x3f, 0x00, 0x00, 0x00,
                                       0xbe, 0x80, 0x00, 0x00,
                                       0x3f, 0x80, 0x00, 0x00}));
  EXPECT_EQ(decode_velocity(velocity_bytes), velocity);
  EXPECT_EQ(decode_mode(encode_mode({0xA104U})), ModePayload{0xA104U});
  EXPECT_EQ(decode_estop(encode_estop({true})), EstopPayload{true});
  const AckPayload ack{77, 0x8008U, "ok"};
  EXPECT_EQ(decode_ack(encode_ack(ack)), ack);
  const ErrorPayload error{77, BridgeError::FeatureUnavailable, "joint"};
  EXPECT_EQ(decode_error(encode_error(error)), error);
}

TEST(PayloadCodec, RoundTripsTelemetryAndCamera) {
  ImuPayload imu{};
  imu.device_time = 123;
  imu.acceleration = {1, 2, 3};
  imu.angular_velocity = {4, 5, 6};
  imu.quaternion = {0, 0, 0, 1};
  EXPECT_EQ(decode_imu(encode_imu(imu)), imu);

  SportPayload sport{124, {1, 2, 3, 4}, 0xB104U};
  EXPECT_EQ(decode_sport(encode_sport(sport)), sport);

  OdometryPayload odom{};
  odom.device_time = 125;
  odom.position = {1, -2, 0.5};
  odom.orientation = {0, 0, 0, 1};
  EXPECT_EQ(decode_odometry(encode_odometry(odom)), odom);

  RobotStatePayload state{};
  state.sdk_linked = true;
  state.control_authority = true;
  state.battery_percentage = 88.5F;
  state.wheel_speed = {1, 2, 3, 4};
  EXPECT_EQ(decode_robot_state(encode_robot_state(state)), state);

  CameraChunkPayload camera{2, 123456, 10, {0, 0, 1, 0x67}};
  const auto camera_bytes = encode_camera_chunk(camera);
  EXPECT_EQ(camera_bytes,
            (std::vector<std::uint8_t>{
                0, 0, 0, 2, 0, 0, 0, 0, 0, 1, 0xE2, 0x40,
                0, 0, 0, 10, 0, 0, 1, 0x67}));
  EXPECT_EQ(decode_camera_chunk(camera_bytes), camera);
}

TEST(PayloadCodec, RejectsMalformedAndNonFinitePayloads) {
  EXPECT_THROW(decode_velocity({0, 1}), ProtocolError);
  EXPECT_THROW(encode_velocity(
                   {std::numeric_limits<float>::infinity(), 0, 0}),
               ProtocolError);
  auto hello = encode_hello({1, 1, 0, 1});
  hello[2] = 1;
  EXPECT_THROW(decode_hello(hello), ProtocolError);
  EXPECT_THROW(decode_hello(encode_hello({1, 1, 0x80000000U, 1})),
               ProtocolError);
}

TEST(Sw01Udp, ParsesPackedOdometryAndNormalizesQuaternion) {
  const auto odom = parse_odometry_packet(
      packed_odometry_fixture(), PackingMode::Auto, 1e-6, 1e-6);
  ASSERT_TRUE(odom.has_value());
  EXPECT_DOUBLE_EQ(odom->position[0], 1.0);
  EXPECT_DOUBLE_EQ(odom->position[1], -2.0);
  EXPECT_DOUBLE_EQ(odom->position[2], 0.5);
  EXPECT_DOUBLE_EQ(odom->orientation[3], 1.0);
}

TEST(Sw01Udp, AcceptsNaturalAlignmentAndRejectsBadTail) {
  auto bytes = aligned_odometry_fixture();
  EXPECT_TRUE(parse_odometry_packet(bytes, PackingMode::Auto, 1.0, 1.0));
  bytes[73] = 0;
  EXPECT_FALSE(parse_odometry_packet(bytes, PackingMode::Auto, 1.0, 1.0));
}

TEST(Sw01Udp, RejectsInvalidScaleAndZeroQuaternion) {
  auto bytes = packed_odometry_fixture();
  EXPECT_FALSE(parse_odometry_packet(bytes, PackingMode::Auto, 0.0, 1e-6));
  for (std::size_t offset = 34; offset < 66; ++offset) {
    bytes[offset] = 0;
  }
  EXPECT_FALSE(parse_odometry_packet(bytes, PackingMode::Auto, 1e-6, 1e-6));
}

TEST(Sw01Udp, ParsesPointCloudAndRejectsCountAboveFifty) {
  const auto cloud =
      parse_point_cloud_packet(point_cloud_fixture(1), PackingMode::Auto, 1e-3);
  ASSERT_TRUE(cloud.has_value());
  ASSERT_EQ(cloud->points.size(), 1U);
  EXPECT_DOUBLE_EQ(cloud->points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(cloud->points[0].y, -2.0);
  EXPECT_EQ(cloud->points[0].rgba[0], 0x11223344U);

  const auto oversized = point_cloud_fixture(51);
  EXPECT_FALSE(
      parse_point_cloud_packet(oversized, PackingMode::Auto, 1e-3));
}

}  // namespace
}  // namespace hypertron_ros2_bridge
