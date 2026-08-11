#include <array>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/protocol_handler.hpp"

namespace hypertron_ros2_bridge {
namespace {

Frame ping(std::uint32_t sequence) {
  return {MessageType::Ping, sequence, 1000U + sequence, {}};
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

  CameraChunkPayload camera{2, 10, 123456, true, {0, 0, 1, 0x67}};
  EXPECT_EQ(decode_camera_chunk(encode_camera_chunk(camera)), camera);
}

TEST(PayloadCodec, RejectsMalformedAndNonFinitePayloads) {
  EXPECT_THROW(decode_velocity({0, 1}), ProtocolError);
  EXPECT_THROW(encode_velocity(
                   {std::numeric_limits<float>::infinity(), 0, 0}),
               ProtocolError);
  auto hello = encode_hello({1, 1, 0, 1});
  hello[2] = 1;
  EXPECT_THROW(decode_hello(hello), ProtocolError);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
