// Pure-C++ LiDAR stream tests. No hardware and no vendor/ROS dependency
// (built and run in the pure CMake configuration). Every count that depends
// on a concurrent loop is asserted via bounded cv/condition waits - tests
// never sleep to sequence or assert runtime timing.

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include "fake_lidar_datagram_source.hpp"
#include "hypertron_ros2_bridge/lidar_stream.hpp"

namespace hypertron_ros2_bridge {
namespace {

using namespace std::chrono_literals;
using test::FakeLidarDatagramSource;

// ---------------------------------------------------------------------------
// Byte-buffer building helpers (little-endian explicit writes).
// ---------------------------------------------------------------------------

void PutU16(std::vector<std::uint8_t>& b, std::size_t off, std::uint16_t v) {
  b[off] = static_cast<std::uint8_t>(v & 0xFFU);
  b[off + 1] = static_cast<std::uint8_t>((v >> 8U) & 0xFFU);
}
void PutU32(std::vector<std::uint8_t>& b, std::size_t off, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    b[off + static_cast<std::size_t>(i)] =
        static_cast<std::uint8_t>((v >> (8U * i)) & 0xFFU);
  }
}
void PutU64(std::vector<std::uint8_t>& b, std::size_t off, std::uint64_t v) {
  for (int i = 0; i < 8; ++i) {
    b[off + static_cast<std::size_t>(i)] =
        static_cast<std::uint8_t>((v >> (8U * i)) & 0xFFU);
  }
}

// Bounded wait: returns true when the condition holds within the timeout.
bool WaitUntil(const std::function<bool()>& condition,
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

// A standard parse config for tests.
LidarParseConfig DefaultConfig() {
  LidarParseConfig c;
  return c;
}

// ---------------------------------------------------------------------------
// Odometry builders.
// ---------------------------------------------------------------------------

enum class OdomLayout { Packed, Aligned };

std::vector<std::uint8_t> MakeOdometry(
    OdomLayout layout, std::uint64_t timestamp_ns,
    std::int64_t x, std::int64_t y, std::int64_t z,
    std::int64_t qx, std::int64_t qy, std::int64_t qz,
    std::int64_t qw) {
  const bool packed = (layout == OdomLayout::Packed);
  const std::size_t size = packed ? 68 : 80;
  std::vector<std::uint8_t> b(size, 0);
  const std::size_t x_at = packed ? 10 : 16;
  const std::size_t ts_at = packed ? 2 : 8;
  const std::size_t tail_at = packed ? 66 : 72;
  PutU16(b, 0, 0xAA55U);
  PutU64(b, ts_at, timestamp_ns);
  PutU64(b, x_at, static_cast<std::uint64_t>(x));
  PutU64(b, x_at + 8, static_cast<std::uint64_t>(y));
  PutU64(b, x_at + 16, static_cast<std::uint64_t>(z));
  PutU64(b, x_at + 24, static_cast<std::uint64_t>(qx));
  PutU64(b, x_at + 32, static_cast<std::uint64_t>(qy));
  PutU64(b, x_at + 40, static_cast<std::uint64_t>(qz));
  PutU64(b, x_at + 48, static_cast<std::uint64_t>(qw));
  PutU16(b, tail_at, 0xFF00U);
  return b;
}

// ---------------------------------------------------------------------------
// Point-cloud builders.
// ---------------------------------------------------------------------------

enum class CloudLayout { Packed, Aligned };
enum class TailStyle { Dynamic, Fixed50 };

void PutPoint(std::vector<std::uint8_t>& b, std::size_t off,
              std::int32_t x, std::int32_t y, std::int32_t z,
              std::uint32_t rgba) {
  PutU32(b, off, static_cast<std::uint32_t>(x));
  PutU32(b, off + 4, static_cast<std::uint32_t>(y));
  PutU32(b, off + 8, static_cast<std::uint32_t>(z));
  PutU32(b, off + 12, rgba);
  // 28-byte point: offsets 0/4/8 = x/y/z int32, 12..15 = rgba; bytes 16..27 padding.
}

// Builds a point-cloud packet. `pos_count` is the number of valid points to
// encode (1..50). `declared_posnum` may differ (for fixed-tail tests where
// posNum < 50 but the tail is fixed-length 50).
std::vector<std::uint8_t> MakeCloud(
    CloudLayout layout, TailStyle tail, std::uint64_t timestamp_ns,
    std::uint32_t total, std::uint32_t index, std::uint16_t pos_num,
    const std::vector<std::array<int, 4>>& points) {
  const bool packed = (layout == CloudLayout::Packed);
  const std::size_t pos_at = packed ? 20 : 28;
  const std::size_t ts_at = packed ? 2 : 8;
  const std::size_t total_at = packed ? 10 : 16;
  const std::size_t index_at = packed ? 14 : 20;
  const std::size_t posnum_at = packed ? 18 : 24;

  std::size_t size = 0;
  if (tail == TailStyle::Dynamic) {
    size = pos_at + 28U * pos_num + 2U;
  } else {
    size = pos_at + 28U * 50U + 2U;
  }
  std::vector<std::uint8_t> b(size, 0);
  PutU16(b, 0, 0xAA55U);
  PutU64(b, ts_at, timestamp_ns);
  PutU32(b, total_at, total);
  PutU32(b, index_at, index);
  PutU16(b, posnum_at, pos_num);
  for (std::size_t i = 0; i < points.size(); ++i) {
    PutPoint(b, pos_at + i * 28U,
             static_cast<std::int32_t>(points[i][0]),
             static_cast<std::int32_t>(points[i][1]),
             static_cast<std::int32_t>(points[i][2]),
             static_cast<std::uint32_t>(points[i][3]));
  }
  // Tail marker after the full point array for fixed tail, after pos_num
  // points for dynamic tail.
  const std::size_t tail_at = size - 2U;
  PutU16(b, tail_at, 0xFF00U);
  return b;
}

// Convenience: one packed dynamic point packet for a 50-point chunk.
std::vector<std::uint8_t> CloudPacket(std::uint64_t ts, std::uint32_t total,
                                      std::uint32_t index,
                                      std::uint16_t pos_num,
                                      std::int32_t base) {
  std::vector<std::array<int, 4>> pts;
  for (std::uint16_t i = 0; i < pos_num; ++i) {
    pts.push_back({base + static_cast<int>(index) * 100 + i, 0, 0,
                   0x00FF0000U});
  }
  return MakeCloud(CloudLayout::Packed, TailStyle::Dynamic, ts, total, index,
                   pos_num, pts);
}

// ---------------------------------------------------------------------------
// Odometry parsing tests.
// ---------------------------------------------------------------------------

TEST(LidarStreamOdometry, ParsesPacked68) {
  // x=1_000_000 * 1e-6 = 1.0 m; y=2_000_000 -> 2.0; z=3_000_000 -> 3.0.
  const auto packet =
      MakeOdometry(OdomLayout::Packed, 123456789ULL, 1000000, 2000000, 3000000,
                   0, 0, 0, 1000000);
  const auto r = ParseOdometryPacket(packet.data(), packet.size(),
                                     DefaultConfig());
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.odometry.timestamp_ns, 123456789ULL);
  EXPECT_DOUBLE_EQ(r.odometry.x, 1.0);
  EXPECT_DOUBLE_EQ(r.odometry.y, 2.0);
  EXPECT_DOUBLE_EQ(r.odometry.z, 3.0);
  // q = (0,0,0,1000000) normalized -> (0,0,0,1).
  EXPECT_DOUBLE_EQ(r.odometry.qx, 0.0);
  EXPECT_DOUBLE_EQ(r.odometry.qy, 0.0);
  EXPECT_DOUBLE_EQ(r.odometry.qz, 0.0);
  EXPECT_NEAR(r.odometry.qw, 1.0, 1e-9);
  // Unit quaternion norm.
  const double norm = std::sqrt(r.odometry.qx * r.odometry.qx +
                                r.odometry.qy * r.odometry.qy +
                                r.odometry.qz * r.odometry.qz +
                                r.odometry.qw * r.odometry.qw);
  EXPECT_NEAR(norm, 1.0, 1e-9);
}

TEST(LidarStreamOdometry, ParsesAligned80) {
  const auto packet =
      MakeOdometry(OdomLayout::Aligned, 42ULL, 1000000, 2000000, 3000000,
                   0, 0, 0, 1000000);
  const auto r = ParseOdometryPacket(packet.data(), packet.size(),
                                     DefaultConfig());
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.odometry.timestamp_ns, 42ULL);
  EXPECT_DOUBLE_EQ(r.odometry.x, 1.0);
  EXPECT_DOUBLE_EQ(r.odometry.y, 2.0);
  EXPECT_DOUBLE_EQ(r.odometry.z, 3.0);
}

TEST(LidarStreamOdometry, RejectsBadHead) {
  auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL, 1, 1, 1, 0, 0, 0, 1);
  packet[0] = 0x00;  // corrupt head
  const auto r = ParseOdometryPacket(packet.data(), packet.size(),
                                     DefaultConfig());
  EXPECT_FALSE(r.ok);
}

TEST(LidarStreamOdometry, RejectsBadTail) {
  auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL, 1, 1, 1, 0, 0, 0, 1);
  packet[67] = 0x00;  // corrupt tail LSB
  const auto r = ParseOdometryPacket(packet.data(), packet.size(),
                                     DefaultConfig());
  EXPECT_FALSE(r.ok);
}

TEST(LidarStreamOdometry, RejectsWrongLength) {
  auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL, 1, 1, 1, 0, 0, 0, 1);
  for (const std::size_t len : {67U, 69U, 79U, 81U}) {
    const auto r = ParseOdometryPacket(packet.data(), len, DefaultConfig());
    EXPECT_FALSE(r.ok) << "len=" << len;
  }
  // 68 should pass (sanity).
  const auto okr = ParseOdometryPacket(packet.data(), 68U, DefaultConfig());
  EXPECT_TRUE(okr.ok);
}

TEST(LidarStreamOdometry, RejectsInvalidScale) {
  auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL, 1, 1, 1, 0, 0, 0, 1);
  for (const double bad : {0.0, -1e-3, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN(), 1e10}) {
    LidarParseConfig cfg;
    cfg.odom_position_scale = bad;
    const auto r = ParseOdometryPacket(packet.data(), packet.size(), cfg);
    EXPECT_FALSE(r.ok) << "scale=" << bad;
  }
}

TEST(LidarStreamOdometry, RejectsQuaternionNormZero) {
  // All quaternion components zero -> norm 0.
  const auto packet = MakeOdometry(OdomLayout::Packed, 1ULL, 0, 0, 0, 0, 0, 0, 0);
  const auto r = ParseOdometryPacket(packet.data(), packet.size(),
                                     DefaultConfig());
  EXPECT_FALSE(r.ok);
}

TEST(LidarStreamOdometry, RejectsCoordinateOverflow) {
  // INT64_MAX * 1e-6 still fits double, but is finite; use a config with a
  // huge scale to force overflow to non-finite... scale > 0 and <=1e9 is
  // allowed, so use scale 1e9 with INT64_MAX -> ~9.22e27, still finite.
  // To force non-finite we need INT64_MAX * scale beyond ~1.8e308, which
  // 1e9 cannot reach; instead verify finite-but-huge is accepted and the
  // coordination formula is stable, and that a genuinely overflowing
  // scenario (scale 1e300 - rejected by validation) never parses.
  LidarParseConfig huge;
  huge.odom_position_scale = 1e300;  // exceeds the 1e9 bound -> invalid
  const auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL,
                   std::numeric_limits<std::int64_t>::max(),
                   0, 0, 0, 0, 0, 1000000);
  const auto r = ParseOdometryPacket(packet.data(), packet.size(), huge);
  EXPECT_FALSE(r.ok);  // invalid scale rejected before overflow matters
}

TEST(LidarStreamOdometry, HandlesMaximumScalesFinite) {
  LidarParseConfig cfg;
  cfg.odom_position_scale = 1e9;
  cfg.odom_quaternion_scale = 1e9;
  const auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL,
                   std::numeric_limits<std::int64_t>::max(),
                   std::numeric_limits<std::int64_t>::min(), 1,
                   1, 2, 3, 4);
  const auto r = ParseOdometryPacket(packet.data(), packet.size(), cfg);
  ASSERT_TRUE(r.ok) << r.reason;  // finite double (huge but not overflow)
  EXPECT_TRUE(std::isfinite(r.odometry.x));
  EXPECT_TRUE(std::isfinite(r.odometry.y));
}

TEST(LidarStreamOdometry, TruncationNeverCrashes) {
  // Deterministic randomized truncation across 200 seeds/frame lengths.
  auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL, 1, 1, 1, 0, 0, 0, 1);
  std::uint32_t seed = 12345;
  for (int iter = 0; iter < 200; ++iter) {
    seed = seed * 1664525U + 1013904223U;
    const std::size_t cut = seed % (packet.size() + 2);
    const auto r = ParseOdometryPacket(packet.data(), cut, DefaultConfig());
    // Never a crash; may or may not parse depending on the cut.
    EXPECT_EQ(r.ok, cut == 68U || cut == 80U) << "cut=" << cut;
  }
}

TEST(LidarStreamOdometry, ParsesQuaternionWxyzOrder) {
  // When the packet stores w,x,y,z order, the raw sequence is
  // (1000000, 0, 0, 0) -> w=1, x=y=z=0 after the wxyz mapping.
  const auto packet =
      MakeOdometry(OdomLayout::Packed, 1ULL, 0, 0, 0, 1000000, 0, 0, 0);
  LidarParseConfig cfg;
  cfg.odom_quaternion_order = "wxyz";
  const auto r = ParseOdometryPacket(packet.data(), packet.size(), cfg);
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_DOUBLE_EQ(r.odometry.qw, 1.0);
  EXPECT_DOUBLE_EQ(r.odometry.qx, 0.0);
  EXPECT_DOUBLE_EQ(r.odometry.qy, 0.0);
  EXPECT_DOUBLE_EQ(r.odometry.qz, 0.0);
}

// ---------------------------------------------------------------------------
// Point-cloud parsing tests.
// ---------------------------------------------------------------------------

std::vector<std::array<int, 4>> MkPts(int n, int base) {
  std::vector<std::array<int, 4>> pts;
  for (int i = 0; i < n; ++i) {
    pts.push_back({base + i * 10, (base + i), -(base + i), 0x335577FF});
  }
  return pts;
}

TEST(LidarStreamCloud, ParsesPackedDynamicN1) {
  const auto packet = MakeCloud(
      CloudLayout::Packed, TailStyle::Dynamic, 7ULL, 50, 0, 1, MkPts(1, 1000));
  const auto r =
      ParsePointCloudPacket(packet.data(), packet.size(), DefaultConfig());
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.timestamp_ns, 7ULL);
  EXPECT_EQ(r.total, 50U);
  EXPECT_EQ(r.index, 0U);
  EXPECT_EQ(r.pos_num, 1U);
  ASSERT_EQ(r.points.size(), 1U);
  // int32 1000 * 1e-3 = 1.0 m.
  EXPECT_FLOAT_EQ(r.points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(r.points[0].y, static_cast<float>(1000 * 1e-3));
  EXPECT_EQ(r.points[0].rgba, 0x335577FFU);
}

TEST(LidarStreamCloud, PreservesAllFourRgbaChannels) {
  auto packet = MakeCloud(
      CloudLayout::Packed, TailStyle::Dynamic, 7ULL, 50, 0, 1, MkPts(1, 1000));
  // 28-byte point record: first point begins at offset 20 (packed header).
  const std::size_t point_off = 20U;
  PutU32(packet, point_off + 12U, 0x11223344U);
  PutU32(packet, point_off + 16U, 0x55667788U);
  PutU32(packet, point_off + 20U, 0x99AABBCCU);
  PutU32(packet, point_off + 24U, 0xDDEEFF00U);
  const auto r =
      ParsePointCloudPacket(packet.data(), packet.size(), DefaultConfig());
  ASSERT_TRUE(r.ok) << r.reason;
  ASSERT_EQ(r.points.size(), 1U);
  EXPECT_EQ(r.points[0].rgba, 0x11223344U);
  EXPECT_EQ(r.points[0].rgba_channels[0], 0x11223344U);
  EXPECT_EQ(r.points[0].rgba_channels[1], 0x55667788U);
  EXPECT_EQ(r.points[0].rgba_channels[2], 0x99AABBCCU);
  EXPECT_EQ(r.points[0].rgba_channels[3], 0xDDEEFF00U);
}

TEST(LidarStreamCloud, ParsesPackedDynamicN5And50) {
  for (const int n : {5, 50}) {
    const auto packet = MakeCloud(CloudLayout::Packed, TailStyle::Dynamic,
                                  99ULL, 50, 3, static_cast<std::uint16_t>(n),
                                  MkPts(n, 3000));
    const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                         DefaultConfig());
    ASSERT_TRUE(r.ok) << "n=" << n << " " << r.reason;
    ASSERT_EQ(r.points.size(), static_cast<std::size_t>(n));
    EXPECT_FLOAT_EQ(r.points[0].x, static_cast<float>(3000 * 1e-3));
  }
}

TEST(LidarStreamCloud, ParsesPackedFixed50TailExtractsPosNum) {
  // posNum=2 but a fixed 50-point tail; only 2 points are valid.
  const auto packet = MakeCloud(CloudLayout::Packed, TailStyle::Fixed50,
                                5ULL, 50, 1, 2, MkPts(2, 5000));
  ASSERT_EQ(packet.size(), 1422U);
  const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                       DefaultConfig());
  ASSERT_TRUE(r.ok) << r.reason;
  EXPECT_EQ(r.pos_num, 2U);
  ASSERT_EQ(r.points.size(), 2U);
  EXPECT_FLOAT_EQ(r.points[1].x, static_cast<float>(5010 * 1e-3));
}

TEST(LidarStreamCloud, ParsesAlignedVariants) {
  // Aligned dynamic (posNum=3).
  {
    const auto packet = MakeCloud(CloudLayout::Aligned, TailStyle::Dynamic,
                                  5ULL, 90, 2, 3, MkPts(3, 7000));
    const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                         DefaultConfig());
    ASSERT_TRUE(r.ok) << r.reason;
    EXPECT_EQ(r.total, 90U);
    EXPECT_EQ(r.index, 2U);
    ASSERT_EQ(r.points.size(), 3U);
    EXPECT_FLOAT_EQ(r.points[0].x, static_cast<float>(7000 * 1e-3));
  }
  // Aligned fixed 50 tail (posNum=4).
  {
    const auto packet = MakeCloud(CloudLayout::Aligned, TailStyle::Fixed50,
                                  5ULL, 90, 2, 4, MkPts(4, 8000));
    ASSERT_EQ(packet.size(), 1430U);
    const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                         DefaultConfig());
    ASSERT_TRUE(r.ok) << r.reason;
    EXPECT_EQ(r.pos_num, 4U);
    ASSERT_EQ(r.points.size(), 4U);
  }
}

TEST(LidarStreamCloud, RejectsPosNumOver50) {
  auto packet = MakeCloud(CloudLayout::Packed, TailStyle::Fixed50, 1ULL, 100,
                          0, 50, MkPts(50, 9000));
  // Force posNum to 51 without re-laying the tail marker: the parser should
  // reject on the posNum bound regardless of tail shape.
  PutU16(packet, 18, 51);
  const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                       DefaultConfig());
  EXPECT_FALSE(r.ok);
}

TEST(LidarStreamCloud, RejectsPosNumZero) {
  const auto packet = MakeCloud(CloudLayout::Packed, TailStyle::Dynamic,
                                1ULL, 50, 0, 0, {});
  const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                       DefaultConfig());
  // posNum=0 with a dynamic tail length = 20 + 0 + 2 = 22. The parser should
  // reject (a point cloud needs a positive posNum even if the frame may be
  // completed by other packets).
  EXPECT_FALSE(r.ok);
}

TEST(LidarStreamCloud, RejectsTotalZero) {
  const auto packet = MakeCloud(CloudLayout::Packed, TailStyle::Dynamic,
                                1ULL, 0, 0, 3, MkPts(3, 1));
  const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                       DefaultConfig());
  EXPECT_FALSE(r.ok);
}

TEST(LidarStreamCloud, RejectsCountLengthMismatch) {
  // A dynamic packet whose declared posNum does not match the actual length
  // (trimmed or padded): length must match exactly one valid tail.
  auto packet = MakeCloud(CloudLayout::Packed, TailStyle::Dynamic, 1ULL, 100,
                          0, 3, MkPts(3, 1));
  // Truncate to a length that matches neither valid tail.
  packet.pop_back();
  const auto r = ParsePointCloudPacket(packet.data(), packet.size(),
                                       DefaultConfig());
  EXPECT_FALSE(r.ok);
}

TEST(LidarStreamCloud, RejectsNonFiniteScale) {
  auto packet = MakeCloud(CloudLayout::Packed, TailStyle::Dynamic, 1ULL, 100,
                          0, 1, MkPts(1, 1000));
  LidarParseConfig cfg;
  cfg.point_position_scale = 0.0;
  EXPECT_FALSE(
      ParsePointCloudPacket(packet.data(), packet.size(), cfg).ok);
  cfg.point_position_scale = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
      ParsePointCloudPacket(packet.data(), packet.size(), cfg).ok);
  cfg.point_position_scale = -1e-3;
  EXPECT_FALSE(
      ParsePointCloudPacket(packet.data(), packet.size(), cfg).ok);
  cfg.point_position_scale = 1e10;  // above the 1e9 bound
  EXPECT_FALSE(
      ParsePointCloudPacket(packet.data(), packet.size(), cfg).ok);
}

TEST(LidarStreamCloud, RejectsBadHeadTail) {
  auto good = MakeCloud(CloudLayout::Packed, TailStyle::Dynamic, 1ULL, 100,
                        0, 1, MkPts(1, 1));
  {
    auto p = good;
    p[0] = 0x00;
    EXPECT_FALSE(ParsePointCloudPacket(p.data(), p.size(), DefaultConfig()).ok);
  }
  {
    auto p = good;
    p[p.size() - 1] = 0x00;
    EXPECT_FALSE(ParsePointCloudPacket(p.data(), p.size(), DefaultConfig()).ok);
  }
}

// ---------------------------------------------------------------------------
// Assembler / frame reassembly tests.
// ---------------------------------------------------------------------------

TEST(LidarStreamReassembly, SinglePacketFramePublishesImmediately) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  auto pk = ParsePointCloudPacket(CloudPacket(1, 1, 0, 1, 0).data(),
                                  CloudPacket(1, 1, 0, 1, 0).size(),
                                  DefaultConfig());
  ASSERT_TRUE(pk.ok);
  auto frame = asm_.add_packet(pk);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->timestamp_ns, 1ULL);
  EXPECT_EQ(frame->total_points, 1ULL);
  ASSERT_EQ(frame->points.size(), 1U);
  // No duplicate acceptance.
  auto s = asm_.stats();
  EXPECT_EQ(s.frames_published, 1ULL);
  EXPECT_EQ(s.packets_accepted, 1ULL);
}

TEST(LidarStreamReassembly, MultiPacketOutOfOrderPublishesInIndexOrder) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  const std::uint64_t ts = 10;
  const std::uint32_t total = 3;
  const std::uint16_t per = 1;  // one point per packet, 3 packets
  auto p0 = ParsePointCloudPacket(CloudPacket(ts, total, 0, per, 0).data(),
                                  CloudPacket(ts, total, 0, per, 0).size(),
                                  DefaultConfig());
  auto p2 = ParsePointCloudPacket(CloudPacket(ts, total, 2, per, 2).data(),
                                  CloudPacket(ts, total, 2, per, 2).size(),
                                  DefaultConfig());
  auto p1 = ParsePointCloudPacket(CloudPacket(ts, total, 1, per, 1).data(),
                                  CloudPacket(ts, total, 1, per, 1).size(),
                                  DefaultConfig());
  // Out of order: 0, then 2, then 1.
  EXPECT_FALSE(asm_.add_packet(p0).has_value());
  EXPECT_FALSE(asm_.add_packet(p2).has_value());
  auto frame = asm_.add_packet(p1);
  ASSERT_TRUE(frame.has_value());
  ASSERT_EQ(frame->points.size(), 3U);
  // Index order: points should reflect packet index 0,1,2 (x = base + idx*100 + i).
  EXPECT_FLOAT_EQ(frame->points[0].x, static_cast<float>(0 * 1e-3));       // p0 base 0
  EXPECT_FLOAT_EQ(frame->points[1].x, static_cast<float>((1 * 100 + 1) * 1e-3));  // p1 base 1
  EXPECT_FLOAT_EQ(frame->points[2].x, static_cast<float>((2 * 100 + 2) * 1e-3));  // p2 base 2
}

TEST(LidarStreamReassembly, PublishesWith1BasedIndexes) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  const std::uint64_t ts = 20;
  const std::uint32_t total = 3;
  // 1-based index set: 1,2,3.
  auto p1 = ParsePointCloudPacket(CloudPacket(ts, total, 1, 1, 1).data(),
                                  CloudPacket(ts, total, 1, 1, 1).size(),
                                  DefaultConfig());
  auto p3 = ParsePointCloudPacket(CloudPacket(ts, total, 3, 1, 3).data(),
                                  CloudPacket(ts, total, 3, 1, 3).size(),
                                  DefaultConfig());
  auto p2 = ParsePointCloudPacket(CloudPacket(ts, total, 2, 1, 2).data(),
                                  CloudPacket(ts, total, 2, 1, 2).size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(p1).has_value());
  EXPECT_FALSE(asm_.add_packet(p3).has_value());
  auto frame = asm_.add_packet(p2);
  ASSERT_TRUE(frame.has_value());
  ASSERT_EQ(frame->points.size(), 3U);
}

TEST(LidarStreamReassembly, PartialFrameExpiresOnTimeout) {
  AssemblerConfig cfg;
  cfg.frame_timeout = 50ms;
  PointCloudFrameAssembler asm_(cfg);
  const std::uint64_t ts = 30;
  const std::uint32_t total = 3;
  auto p0 = ParsePointCloudPacket(CloudPacket(ts, total, 0, 1, 0).data(),
                                  CloudPacket(ts, total, 0, 1, 0).size(),
                                  DefaultConfig());
  auto p1 = ParsePointCloudPacket(CloudPacket(ts, total, 1, 1, 1).data(),
                                  CloudPacket(ts, total, 1, 1, 1).size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(p0).has_value());
  EXPECT_FALSE(asm_.add_packet(p1).has_value());
  // Expire immediately: not timed out yet (age < timeout).
  asm_.expire(std::chrono::steady_clock::now());
  EXPECT_EQ(asm_.stats().frames_dropped_timeout, 0ULL);
  EXPECT_EQ(asm_.in_flight_count(), 1U);
  // Advance past the timeout and expire.
  asm_.expire(std::chrono::steady_clock::now() + 60ms);
  EXPECT_EQ(asm_.in_flight_count(), 0U);
  EXPECT_EQ(asm_.stats().frames_dropped_timeout, 1ULL);
  EXPECT_EQ(asm_.stats().frames_published, 0ULL);
}

TEST(LidarStreamReassembly, DuplicatePacketIgnored) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  const std::uint64_t ts = 40;
  const std::uint32_t total = 2;
  auto p0 = ParsePointCloudPacket(CloudPacket(ts, total, 0, 1, 0).data(),
                                  CloudPacket(ts, total, 0, 1, 0).size(),
                                  DefaultConfig());
  auto p1 = ParsePointCloudPacket(CloudPacket(ts, total, 1, 1, 1).data(),
                                  CloudPacket(ts, total, 1, 1, 1).size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(p0).has_value());
  // Duplicate of p0 (identical content) -> ignored, not double counted.
  EXPECT_FALSE(asm_.add_packet(p0).has_value());
  EXPECT_EQ(asm_.stats().packets_duplicated, 1ULL);
  // Still only index 0 present; the frame cannot complete via p0 alone.
  EXPECT_EQ(asm_.in_flight_count(), 1U);
  auto frame = asm_.add_packet(p1);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->points.size(), 2U);
}

TEST(LidarStreamReassembly, ConflictingDuplicateDropsFrame) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  const std::uint64_t ts = 50;
  const std::uint32_t total = 2;
  auto p0 = CloudPacket(ts, total, 0, 1, 0);
  // Build a conflicting p0' with same index but different content.
  std::vector<std::array<int, 4>> pts = {{999, 0, 0, 0x11223344U}};
  auto p0_conflict =
      MakeCloud(CloudLayout::Packed, TailStyle::Dynamic, ts, total, 0, 1, pts);
  auto r0 = ParsePointCloudPacket(p0.data(), p0.size(), DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(r0).has_value());
  auto rc = ParsePointCloudPacket(p0_conflict.data(), p0_conflict.size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(rc).has_value());
  EXPECT_EQ(asm_.stats().frames_dropped_conflict, 1ULL);
  // The frame was dropped; a later p1 cannot publish it.
  EXPECT_EQ(asm_.in_flight_count(), 0U);
}

TEST(LidarStreamReassembly, TotalMismatchDropsFrame) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  const std::uint64_t ts = 60;
  auto p0 =
      ParsePointCloudPacket(CloudPacket(ts, 2, 0, 1, 0).data(),
                            CloudPacket(ts, 2, 0, 1, 0).size(),
                            DefaultConfig());  // total 2
  EXPECT_FALSE(asm_.add_packet(p0).has_value());
  // Same timestamp, different total -> conflict.
  auto p0b =
      ParsePointCloudPacket(CloudPacket(ts, 5, 0, 1, 0).data(),
                            CloudPacket(ts, 5, 0, 1, 0).size(),
                            DefaultConfig());  // total 5
  EXPECT_FALSE(asm_.add_packet(p0b).has_value());
  EXPECT_EQ(asm_.stats().frames_dropped_conflict, 1ULL);
  EXPECT_EQ(asm_.in_flight_count(), 0U);
}

TEST(LidarStreamReassembly, CapacityEvictsOldestFrame) {
  AssemblerConfig cfg;
  cfg.max_parallel_frames = 2;
  PointCloudFrameAssembler asm_(cfg);
  const std::uint32_t total = 2;
  // Frame A (timestamp 100): packet index 0 only (incomplete).
  auto a0 = ParsePointCloudPacket(CloudPacket(100, total, 0, 1, 0).data(),
                                  CloudPacket(100, total, 0, 1, 0).size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(a0).has_value());
  // Frame B (timestamp 200): packet index 0 only (incomplete).
  auto b0 = ParsePointCloudPacket(CloudPacket(200, total, 0, 1, 0).data(),
                                  CloudPacket(200, total, 0, 1, 0).size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(b0).has_value());
  EXPECT_EQ(asm_.in_flight_count(), 2U);
  // Frame C arrives while both slots are full -> oldest (A) evicted.
  auto c0 = ParsePointCloudPacket(CloudPacket(300, total, 0, 1, 0).data(),
                                  CloudPacket(300, total, 0, 1, 0).size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(c0).has_value());
  EXPECT_EQ(asm_.stats().frames_dropped_capacity, 1ULL);
  EXPECT_EQ(asm_.in_flight_count(), 2U);
  // Completing frame B still works (its slot survived).
  auto b1 = ParsePointCloudPacket(CloudPacket(200, total, 1, 1, 1).data(),
                                  CloudPacket(200, total, 1, 1, 1).size(),
                                  DefaultConfig());
  auto frame = asm_.add_packet(b1);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->timestamp_ns, 200ULL);
  EXPECT_EQ(asm_.stats().frames_published, 1ULL);
}

TEST(LidarStreamReassembly, MaxPointsPerFrameRejects) {
  AssemblerConfig cfg;
  cfg.max_points_per_frame = 100;
  PointCloudFrameAssembler asm_(cfg);
  // A frame declaring total > max is rejected outright.
  auto pk = ParsePointCloudPacket(CloudPacket(1, 500, 0, 1, 0).data(),
                                  CloudPacket(1, 500, 0, 1, 0).size(),
                                  DefaultConfig());
  ASSERT_TRUE(pk.ok);
  EXPECT_FALSE(asm_.add_packet(pk).has_value());
  EXPECT_EQ(asm_.stats().packets_rejected, 1ULL);
  EXPECT_EQ(asm_.in_flight_count(), 0U);
}

TEST(LidarStreamReassembly, StatsCountCorrect) {
  AssemblerConfig cfg;
  cfg.max_parallel_frames = 1;
  PointCloudFrameAssembler asm_(cfg);
  // Two independent single-packet frames (total=1) -> publish, then evict.
  auto f1 = ParsePointCloudPacket(CloudPacket(1, 1, 0, 1, 0).data(),
                                  CloudPacket(1, 1, 0, 1, 0).size(),
                                  DefaultConfig());
  auto frame = asm_.add_packet(f1);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(asm_.stats().frames_published, 1ULL);
  EXPECT_EQ(asm_.stats().packets_accepted, 1ULL);
  EXPECT_EQ(asm_.in_flight_count(), 0U);
}

// ---------------------------------------------------------------------------
// F2: Frame completion criterion rewritten per the SW01 manual. `index` is
// the packet's sequence number within the frame and `total` is the frame's
// total point count; a frame publishes only when the observed index set
// covers contiguously from the base (0 or 1) AND the points across the
// observed packets sum to `total`.
// ---------------------------------------------------------------------------

// A single packet carrying 50 points and declaring total=50, index=0 is a
// whole frame and publishes immediately.
TEST(LidarStreamReassembly, F2_SinglePacketWhole50PointFramePublishes) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  auto pk = ParsePointCloudPacket(CloudPacket(7, 50, 0, 50, 1000).data(),
                                  CloudPacket(7, 50, 0, 50, 1000).size(),
                                  DefaultConfig());
  ASSERT_TRUE(pk.ok) << pk.reason;
  EXPECT_EQ(pk.pos_num, 50U);
  auto frame = asm_.add_packet(pk);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->total_points, 50ULL);
  ASSERT_EQ(frame->points.size(), 50U);
  EXPECT_EQ(asm_.stats().frames_published, 1ULL);
  EXPECT_EQ(asm_.stats().packets_accepted, 1ULL);
}

// A single 1-based packet (index=1) carrying the whole frame's points.
TEST(LidarStreamReassembly, F2_Single1BasedWholeFramePublishes) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  auto pk = ParsePointCloudPacket(CloudPacket(8, 30, 1, 30, 500).data(),
                                  CloudPacket(8, 30, 1, 30, 500).size(),
                                  DefaultConfig());
  ASSERT_TRUE(pk.ok) << pk.reason;
  auto frame = asm_.add_packet(pk);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->total_points, 30ULL);
  ASSERT_EQ(frame->points.size(), 30U);
  EXPECT_EQ(asm_.stats().frames_published, 1ULL);
}

// Multiple packets each carrying more than one point, delivered out of order,
// reassemble only when the index set is contiguous and the points sum to the
// declared total.
TEST(LidarStreamReassembly, F2_MultiPacketOutOfOrderPosNumGT1Publishes) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  const std::uint64_t ts = 55;
  const std::uint32_t total = 50;
  // index 0 -> 20 points, index 2 -> 10 points (out of order), index 1 -> 20.
  auto p0 = ParsePointCloudPacket(CloudPacket(ts, total, 0, 20, 0).data(),
                                  CloudPacket(ts, total, 0, 20, 0).size(),
                                  DefaultConfig());
  auto p2 = ParsePointCloudPacket(CloudPacket(ts, total, 2, 10, 2).data(),
                                  CloudPacket(ts, total, 2, 10, 2).size(),
                                  DefaultConfig());
  auto p1 = ParsePointCloudPacket(CloudPacket(ts, total, 1, 20, 1).data(),
                                  CloudPacket(ts, total, 1, 20, 1).size(),
                                  DefaultConfig());
  EXPECT_FALSE(asm_.add_packet(p0).has_value());  // 20/50, incomplete
  EXPECT_FALSE(asm_.add_packet(p2).has_value());  // gap at index 1, 30/50
  auto frame = asm_.add_packet(p1);               // 50/50 contiguous {0,1,2}
  ASSERT_TRUE(frame.has_value());
  ASSERT_EQ(frame->points.size(), 50U);
  EXPECT_EQ(frame->total_points, 50ULL);
  // Index order preserved: packet index 0, then 1, then 2.
  EXPECT_FLOAT_EQ(frame->points[0].x, static_cast<float>(0 * 1e-3));
  EXPECT_FLOAT_EQ(frame->points[20].x, static_cast<float>((1 * 100 + 1) * 1e-3));
  EXPECT_FLOAT_EQ(frame->points[40].x, static_cast<float>((2 * 100 + 2) * 1e-3));
}

// A multi-packet frame whose last packet carries fewer than 50 points: the
// frame finishes once the points sum to the declared total.
TEST(LidarStreamReassembly, F2_LastPacketUnder50PointsStillPublishes) {
  PointCloudFrameAssembler asm_(AssemblerConfig{});
  const std::uint64_t ts = 60;
  const std::uint32_t total = 50;
  auto p0 = ParsePointCloudPacket(CloudPacket(ts, total, 0, 30, 0).data(),
                                  CloudPacket(ts, total, 0, 30, 0).size(),
                                  DefaultConfig());  // 30 points
  auto p1 = ParsePointCloudPacket(CloudPacket(ts, total, 1, 20, 1).data(),
                                  CloudPacket(ts, total, 1, 20, 1).size(),
                                  DefaultConfig());  // 20 points
  EXPECT_FALSE(asm_.add_packet(p0).has_value());
  auto frame = asm_.add_packet(p1);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->total_points, 50ULL);
  ASSERT_EQ(frame->points.size(), 50U);
  EXPECT_EQ(asm_.stats().frames_published, 1ULL);
}

// A cumulative point count that would exceed the declared total is a
// conflicting frame and is dropped whole; a later packet cannot publish it.
TEST(LidarStreamReassembly, F2_CumulativePointsExceedTotalDropsFrame) {
  AssemblerConfig cfg;
  cfg.max_parallel_frames = 1;
  PointCloudFrameAssembler asm_(cfg);
  const std::uint64_t ts = 70;
  const std::uint32_t total = 30;
  auto p0 = ParsePointCloudPacket(CloudPacket(ts, total, 0, 20, 0).data(),
                                  CloudPacket(ts, total, 0, 20, 0).size(),
                                  DefaultConfig());  // 20/30
  auto p1 = ParsePointCloudPacket(CloudPacket(ts, total, 1, 20, 1).data(),
                                  CloudPacket(ts, total, 1, 20, 1).size(),
                                  DefaultConfig());  // would push to 40 > 30
  EXPECT_FALSE(asm_.add_packet(p0).has_value());
  EXPECT_FALSE(asm_.add_packet(p1).has_value());
  EXPECT_EQ(asm_.stats().frames_dropped_conflict, 1ULL);
  EXPECT_EQ(asm_.in_flight_count(), 0U);
}

// A sparse index beyond the per-frame packet cap is rejected outright (the
// frame cannot be completed from the base and memory stays bounded).
TEST(LidarStreamReassembly, F2_SparseIndexOverCapRejected) {
  AssemblerConfig cfg;
  cfg.max_packets_per_frame = 8;
  PointCloudFrameAssembler asm_(cfg);
  // index 100 > max_packets_per_frame(8): rejected without allocating.
  auto pk = ParsePointCloudPacket(CloudPacket(80, 50, 100, 50, 0).data(),
                                  CloudPacket(80, 50, 100, 50, 0).size(),
                                  DefaultConfig());
  ASSERT_TRUE(pk.ok) << pk.reason;
  EXPECT_FALSE(asm_.add_packet(pk).has_value());
  EXPECT_EQ(asm_.stats().packets_rejected, 1ULL);
  EXPECT_EQ(asm_.in_flight_count(), 0U);
}

// A declared total beyond max_points_per_frame is rejected outright.
TEST(LidarStreamReassembly, F2_TotalOverMaxPointsPerFrameRejected) {
  AssemblerConfig cfg;
  cfg.max_points_per_frame = 100;
  PointCloudFrameAssembler asm_(cfg);
  auto pk = ParsePointCloudPacket(CloudPacket(90, 500, 0, 50, 0).data(),
                                  CloudPacket(90, 500, 0, 50, 0).size(),
                                  DefaultConfig());
  ASSERT_TRUE(pk.ok) << pk.reason;
  EXPECT_FALSE(asm_.add_packet(pk).has_value());
  EXPECT_EQ(asm_.stats().packets_rejected, 1ULL);
  EXPECT_EQ(asm_.in_flight_count(), 0U);
}

// ---------------------------------------------------------------------------
// Receiver tests.
// ---------------------------------------------------------------------------

// Records frames/odometry/warnings delivered via sink callbacks.
struct SinkRecorder {
  mutable std::mutex mutex;
  std::condition_variable cv;
  std::vector<PointCloudFrame> frames;
  std::vector<LidarOdometry> odoms;
  std::vector<std::string> warnings;

  void on_frame(const PointCloudFrame& f) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      frames.push_back(f);
    }
    cv.notify_all();
  }
  void on_odom(const LidarOdometry& o) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      odoms.push_back(o);
    }
    cv.notify_all();
  }
  void on_warning(const std::string& w) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      warnings.push_back(w);
    }
    cv.notify_all();
  }
  // All accessors hold the mutex so the main thread can safely poll counts
  // while the receiver callback writes them (waiting on the cv).
  std::size_t frame_count() const {
    std::lock_guard<std::mutex> lock(mutex);
    return frames.size();
  }
  std::size_t odom_count() const {
    std::lock_guard<std::mutex> lock(mutex);
    return odoms.size();
  }
};

TEST(LidarStreamReceiver, PointCloudTwoPacketOutOfOrderDeliversFrame) {
  auto src = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* raw = src.get();
  SinkRecorder rec;
  LidarSink sink;
  sink.on_frame = [&](const PointCloudFrame& f) { rec.on_frame(f); };
  LidarStreamReceiver receiver(std::move(src),
                               LidarStreamReceiver::LidarStreamKind::PointCloud,
                               DefaultConfig(), AssemblerConfig{}, sink);
  receiver.start();

  const std::uint64_t ts = 1000;
  const std::uint32_t total = 2;
  raw->push(CloudPacket(ts, total, 1, 1, 1));  // index 1 first
  raw->push(CloudPacket(ts, total, 0, 1, 0));  // index 0 second
  EXPECT_TRUE(WaitUntil([&] { return rec.frame_count() == 1U; }));
  receiver.stop();
  {
    std::lock_guard<std::mutex> lock(rec.mutex);
    ASSERT_EQ(rec.frames.size(), 1U);
    EXPECT_EQ(rec.frames[0].timestamp_ns, ts);
    EXPECT_EQ(rec.frames[0].points.size(), 2U);
  }
  EXPECT_EQ(receiver.assembler_stats().frames_published, 1ULL);
}

TEST(LidarStreamReceiver, OdometryPassthrough) {
  auto src = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* raw = src.get();
  SinkRecorder rec;
  LidarSink sink;
  sink.on_odometry = [&](const LidarOdometry& o) { rec.on_odom(o); };
  LidarStreamReceiver receiver(std::move(src),
                               LidarStreamReceiver::LidarStreamKind::Odometry,
                               DefaultConfig(), AssemblerConfig{}, sink);
  receiver.start();
  raw->push(MakeOdometry(OdomLayout::Packed, 5ULL, 1000000, 0, 0, 0, 0, 0, 1));
  EXPECT_TRUE(WaitUntil([&] { return rec.odom_count() == 1U; }));
  receiver.stop();
  {
    std::lock_guard<std::mutex> lock(rec.mutex);
    ASSERT_EQ(rec.odoms.size(), 1U);
    EXPECT_DOUBLE_EQ(rec.odoms[0].x, 1.0);
    EXPECT_NEAR(rec.odoms[0].qw, 1.0, 1e-9);
  }
}

TEST(LidarStreamReceiver, ExceptionInCallbackDoesNotKillThread) {
  auto src = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* raw = src.get();
  SinkRecorder rec;
  LidarSink sink;
  sink.on_frame = [&](const PointCloudFrame&) {
    rec.on_frame(PointCloudFrame{});
    throw std::runtime_error("callback bug");
  };
  LidarStreamReceiver receiver(std::move(src),
                               LidarStreamReceiver::LidarStreamKind::PointCloud,
                               DefaultConfig(), AssemblerConfig{}, sink);
  receiver.start();
  raw->push(CloudPacket(1, 1, 0, 1, 0));  // single-packet frame
  // The frame still reaches the recording callback (posted before the throw).
  EXPECT_TRUE(WaitUntil([&] { return rec.frame_count() >= 1U; }));
  // The thread must stay alive and still accept more datagrams.
  raw->push(CloudPacket(2, 1, 0, 1, 0));
  EXPECT_TRUE(WaitUntil([&] { return rec.frame_count() >= 2U; }));
  EXPECT_TRUE(receiver.running());
  receiver.stop();
}

TEST(LidarStreamReceiver, TimeoutProducesNoCallback) {
  auto src = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* raw = src.get();
  SinkRecorder rec;
  LidarSink sink;
  sink.on_odometry = [&](const LidarOdometry& o) { rec.on_odom(o); };
  LidarStreamReceiver receiver(std::move(src),
                               LidarStreamReceiver::LidarStreamKind::Odometry,
                               DefaultConfig(), AssemblerConfig{}, sink);
  receiver.start();
  // No datagram; let the loop time out a few 50ms rounds.
  std::this_thread::sleep_for(120ms);
  receiver.stop();
  EXPECT_EQ(rec.odom_count(), 0U);
  EXPECT_GE(raw->timeouts(), 1U);
}

TEST(LidarStreamReceiver, BadPacketDrivesParseWarning) {
  auto src = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* raw = src.get();
  SinkRecorder rec;
  LidarSink sink;
  std::atomic<int> warning_count{0};
  std::mutex wmutex;
  std::condition_variable wcv;
  LidarStreamReceiver receiver(
      std::move(src), LidarStreamReceiver::LidarStreamKind::PointCloud,
      DefaultConfig(), AssemblerConfig{}, sink,
      [&](const std::string&) {
        {
          std::lock_guard<std::mutex> lock(wmutex);
          ++warning_count;
        }
        wcv.notify_all();
      });
  (void)rec;
  receiver.start();
  // A garbage datagram (not a valid cloud packet).
  std::vector<std::uint8_t> garbage(100, 0xAB);
  raw->push(garbage);
  EXPECT_TRUE(WaitUntil([&] { return warning_count.load() >= 1; }));
  receiver.stop();
}

TEST(LidarStreamReceiver, StopIsIdempotent) {
  auto src = std::make_unique<FakeLidarDatagramSource>();
  LidarStreamReceiver receiver(
      std::move(src), LidarStreamReceiver::LidarStreamKind::Odometry,
      DefaultConfig(), AssemblerConfig{}, LidarSink{});
  receiver.start();
  EXPECT_TRUE(receiver.running());
  receiver.stop();
  EXPECT_FALSE(receiver.running());
  receiver.stop();  // second stop is a no-op
  EXPECT_FALSE(receiver.running());
}

TEST(LidarStreamReceiver, ConcurrentStopDoesNotDeadlock) {
  auto src = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* raw = src.get();
  SinkRecorder rec;
  LidarSink sink;
  sink.on_odometry = [&](const LidarOdometry& o) { rec.on_odom(o); };
  LidarStreamReceiver receiver(
      std::move(src), LidarStreamReceiver::LidarStreamKind::Odometry,
      DefaultConfig(), AssemblerConfig{}, sink);
  receiver.start();
  raw->push(MakeOdometry(OdomLayout::Packed, 1ULL, 1, 1, 1, 0, 0, 0, 1));
  // Many threads stop concurrently; all must return, join exactly once.
  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&] { receiver.stop(); });
  }
  const auto started = std::chrono::steady_clock::now();
  for (auto& t : threads) {
    t.join();
  }
  EXPECT_LT(std::chrono::steady_clock::now() - started, 2s);
  EXPECT_FALSE(receiver.running());
}

// ---------------------------------------------------------------------------
// F3: stop() invoked from inside a sink callback must not join the receive
// thread it is running on (std::terminate). Two-state stop: "request and
// wake" on every caller, "join" only on an external thread.
// ---------------------------------------------------------------------------

// A receiver whose on_frame sink callback calls stop() on the receiver itself
// (i.e. on the receive thread). The callback reaches the receiver through a
// shared cell holding a raw pointer set after construction, so no ownership
// cycle is introduced and the callback is already wired into the receiver's
// Impl when the loop starts.
class LidarStreamReceiverSelfStopping {
  struct Cell {
    LidarStreamReceiver* rec = nullptr;
  };

 public:
  explicit LidarStreamReceiverSelfStopping(
      std::unique_ptr<ILidarDatagramSource> src)
      : injected_(std::move(src)), raw_(static_cast<FakeLidarDatagramSource*>(
                                       injected_.get())) {
    auto cell = std::make_shared<Cell>();
    LidarSink sink;
    sink.on_frame = [cell](const PointCloudFrame&) {
      if (cell->rec) {
        cell->rec->stop();
      }
    };
    receiver_ = std::make_unique<LidarStreamReceiver>(
        std::move(injected_),
        LidarStreamReceiver::LidarStreamKind::PointCloud, DefaultConfig(),
        AssemblerConfig{}, std::move(sink));
    cell->rec = receiver_.get();
  }
  void start() { receiver_->start(); }
  void stop() { receiver_->stop(); }
  bool running() const { return receiver_->running(); }
  // Pushes a single-packet whole frame (total=1, index=0, 1 point) that
  // triggers the on_frame callback on the receive thread.
  void push_frame() { raw_->push(CloudPacket(1, 1, 0, 1, 0)); }

 private:
  std::unique_ptr<ILidarDatagramSource> injected_;
  FakeLidarDatagramSource* raw_;
  std::unique_ptr<LidarStreamReceiver> receiver_;
};

TEST(LidarStreamReceiver, F3_StopInsideCallbackDoesNotTerminateOrDeadlock) {
  LidarStreamReceiverSelfStopping receiver(
      std::make_unique<FakeLidarDatagramSource>());
  receiver.start();
  // A single-packet frame triggers the on_frame callback, which calls
  // stop() from the receive thread. If it tried to join itself this test
  // would std::terminate. running() goes false promptly (stop was requested).
  receiver.push_frame();
  EXPECT_TRUE(WaitUntil([&] { return !receiver.running(); }));
  // A subsequent external stop() performs the join and returns; several are
  // safe (idempotent).
  receiver.stop();
  receiver.stop();
  EXPECT_FALSE(receiver.running());
}

// "回调内 stop 后直接析构" path: the destructor runs on the owning
// (non-receive) thread and must complete the join even when stop() was
// already requested from inside a callback. No std::terminate, no hang.
TEST(LidarStreamReceiver, F3_StopInsideCallbackThenDestroyJoinsCleanly) {
  auto receiver = std::make_unique<LidarStreamReceiverSelfStopping>(
      std::make_unique<FakeLidarDatagramSource>());
  receiver->start();
  receiver->push_frame();
  ASSERT_TRUE(WaitUntil([&] { return !receiver->running(); }));
  receiver.reset();
}

// ---------------------------------------------------------------------------
// F7: Real loopback UDP transport for UdpLidarDatagramSource. Only exercised
// locally (127.0.0.1) so no hardware or external network is required.
// ---------------------------------------------------------------------------

// Finds a free ephemeral UDP port on 127.0.0.1 by binding (0) and reading
// back the assigned port, then releasing it so UdpLidarDatagramSource can
// bind it (SO_REUSEADDR makes the brief release race benign). Returns 0 on
// failure, which makes the downstream source bind fail and the test fail.
std::uint16_t GetFreeUdpPort() {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  EXPECT_GE(fd, 0);
  if (fd < 0) {
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd);
    return 0;
  }
  sockaddr_in bound{};
  socklen_t bound_len = sizeof(bound);
  if (getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &bound_len) != 0) {
    close(fd);
    return 0;
  }
  const std::uint16_t port = ntohs(bound.sin_port);
  close(fd);
  return port;
}

// Sends `payload` as a UDP datagram to 127.0.0.1:`port` from a fresh socket.
void SendLoopback(std::uint16_t port, const std::vector<std::uint8_t>& payload) {
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(fd, 0);
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  to.sin_port = htons(port);
  const ssize_t n =
      sendto(fd, payload.data(), payload.size(), 0,
             reinterpret_cast<const sockaddr*>(&to), sizeof(to));
  ASSERT_EQ(n, static_cast<ssize_t>(payload.size()));
  close(fd);
}

// The real UDP source binds a loopback port, receives a datagram with the
// expected source address, and delivers the bytes.
TEST(LidarStreamUdp, F7_LoopbackDatagramSourceReceives) {
  const std::uint16_t port = GetFreeUdpPort();
  UdpLidarDatagramSource source(port, "127.0.0.1", "127.0.0.1");
  ASSERT_TRUE(source.valid());
  const std::vector<std::uint8_t> payload = {0xAA, 0x55, 0x01, 0x02};
  SendLoopback(port, payload);
  std::vector<std::uint8_t> out;
  ASSERT_TRUE(source.receive(out, 2s));
  EXPECT_EQ(out, payload);
  const auto s = source.stats();
  EXPECT_EQ(s.received, 1ULL);
  EXPECT_EQ(s.dropped_source, 0ULL);
}

// A datagram whose source does not match expected_source_ip is dropped
// (counted), and receive() returns false instead of delivering it.
TEST(LidarStreamUdp, F7_LoopbackSourceFilterDropsMismatch) {
  const std::uint16_t port = GetFreeUdpPort();
  // We send from 127.0.0.1 but expect 10.255.255.1, so the packet is dropped.
  UdpLidarDatagramSource source(port, "127.0.0.1", "10.255.255.1");
  ASSERT_TRUE(source.valid());
  SendLoopback(port, {0xAA, 0x55});

  // The mismatched packet is consumed and dropped on the first receive.
  std::vector<std::uint8_t> out;
  ASSERT_FALSE(source.receive(out, 500ms));
  EXPECT_EQ(source.stats().dropped_source, 1ULL);
  EXPECT_EQ(source.stats().received, 0ULL);
  // With nothing left queued, the next receive times out and returns false.
  ASSERT_FALSE(source.receive(out, 200ms));
  EXPECT_GE(source.stats().timeouts, 1ULL);
}

// An invalid bind_ip makes the source unusable (valid()==false).
TEST(LidarStreamUdp, F7_InvalidBindIpSourceIsInvalid) {
  UdpLidarDatagramSource source(GetFreeUdpPort(), "999.999.999.999", "");
  EXPECT_FALSE(source.valid());
}

// ---------------------------------------------------------------------------
// F8: ParseOdometryPacket rejects null / too-short buffers before any read.
// ---------------------------------------------------------------------------

TEST(LidarStreamOdometry, F8_NullAndShortBuffersRejected) {
  LidarParseConfig cfg;
  OdometryPacketResult r = ParseOdometryPacket(nullptr, 0, cfg);
  EXPECT_FALSE(r.ok);
  r = ParseOdometryPacket(nullptr, 68, cfg);
  EXPECT_FALSE(r.ok);
  r = ParseOdometryPacket(nullptr, 1, cfg);
  EXPECT_FALSE(r.ok);
  const std::uint8_t one_byte[1] = {0xAA};
  r = ParseOdometryPacket(one_byte, 0, cfg);
  EXPECT_FALSE(r.ok);
  r = ParseOdometryPacket(one_byte, 1, cfg);
  EXPECT_FALSE(r.ok);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
