#include "hypertron_ros2_bridge/lidar_stream.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>

#include <poll.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace hypertron_ros2_bridge {

namespace {

// ---------------------------------------------------------------------------
// Explicit little-endian reads. All SW01 datagrams are little-endian. These
// never alias memory: they copy bytes into a correctly-typed object and
// convert, avoiding any UB aliasing read.
// ---------------------------------------------------------------------------

std::uint16_t ReadU16LE(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(p[0]) & 0xFFU) |
      (static_cast<std::uint16_t>(p[1]) << 8U));
}

std::uint32_t ReadU32LE(const std::uint8_t* p) {
  return (static_cast<std::uint32_t>(p[0]) & 0xFFU) |
         ((static_cast<std::uint32_t>(p[1]) & 0xFFU) << 8U) |
         ((static_cast<std::uint32_t>(p[2]) & 0xFFU) << 16U) |
         ((static_cast<std::uint32_t>(p[3]) & 0xFFU) << 24U);
}

std::uint64_t ReadU64LE(const std::uint8_t* p) {
  std::uint64_t lo = ReadU32LE(p);
  std::uint64_t hi = ReadU32LE(p + 4);
  return lo | (hi << 32U);
}

std::int32_t ReadI32LE(const std::uint8_t* p) {
  return static_cast<std::int32_t>(ReadU32LE(p));
}

std::int64_t ReadI64LE(const std::uint8_t* p) {
  std::uint64_t lo = ReadU32LE(p);
  std::uint64_t hi = ReadU32LE(p + 4);
  return static_cast<std::int64_t>(lo | (hi << 32U));
}

// Protocol markers.
constexpr std::uint16_t kHeadMarker = 0xAA55U;
constexpr std::uint16_t kTailMarker = 0xFF00U;
constexpr std::uint16_t kMaxPosNum = 50U;

bool IsFinitePositive(double v) {
  return std::isfinite(v) && v > 0.0;
}

bool ValidateScales(const LidarParseConfig& config) {
  return IsFinitePositive(config.point_position_scale) &&
         IsFinitePositive(config.odom_position_scale) &&
         IsFinitePositive(config.odom_quaternion_scale) &&
         std::fabs(config.point_position_scale) <= 1e9 &&
         std::fabs(config.odom_position_scale) <= 1e9 &&
         std::fabs(config.odom_quaternion_scale) <= 1e9;
}

bool ValidQuaternionOrder(const std::string& order) {
  return order == "xyzw" || order == "wxyz";
}

std::string MakeReason(const char* what) {
  return std::string("lidar parse failure: ") + what;
}

//----------------------------------------------------------------------------
// Point-cloud layout descriptions. Two header variants (packed and
// type-aligned), each with a dynamic tail (exactly posNum points) or a fixed
// 50-point tail (posNum points valid, padding remainder ignored).
//----------------------------------------------------------------------------

struct CloudLayout {
  std::size_t timestamp_at;
  std::size_t total_at;
  std::size_t index_at;
  std::size_t posnum_at;
  std::size_t pos_at;
  // Length with the fixed 50-point tail for this header variant.
  std::size_t fixed_len;
};

constexpr CloudLayout kPackedCloud = {2, 10, 14, 18, 20,
                                      20 + 28U * kMaxPosNum + 2U};
constexpr CloudLayout kAlignedCloud = {8, 16, 20, 24, 28,
                                       28 + 28U * kMaxPosNum + 2U};

constexpr std::size_t kPointBytes = 28U;

// Attempts to parse one point-cloud datagram under a single header layout.
// Returns ok=true or a readable reason.
bool ParsePointCloudWithLayout(const std::uint8_t* data, std::size_t len,
                               const CloudLayout& layout,
                               const LidarParseConfig& config,
                               PointCloudPacketResult& result) {
  if (len < layout.posnum_at + 2U) {
    result.reason = MakeReason("datagram too short for header");
    return false;
  }
  const std::uint16_t pos_num = ReadU16LE(data + layout.posnum_at);
  if (pos_num == 0) {
    result.reason = MakeReason("pos_num is zero");
    return false;
  }
  if (pos_num > kMaxPosNum) {
    result.reason = MakeReason("pos_num exceeds the 50-point bound");
    return false;
  }

  const std::size_t dynamic_len = layout.pos_at + kPointBytes * pos_num + 2U;
  std::size_t tail_at{0};
  if (len == dynamic_len) {
    // Dynamic tail: exactly posNum valid points, tail marker at the end.
    tail_at = layout.pos_at + kPointBytes * pos_num;
  } else if (len == layout.fixed_len) {
    // Fixed 50-point tail: only posNum points are meaningful; the padding
    // points are ignored. Tail marker sits after the full 50-point array.
    tail_at = layout.fixed_len - 2U;
  } else {
    result.reason = MakeReason("datagram length matches no valid tail");
    return false;
  }

  if (ReadU16LE(data + tail_at) != kTailMarker) {
    result.reason = MakeReason("tail marker is not 0xFF00");
    return false;
  }

  // Timestamp / total / index.
  result.timestamp_ns = ReadU64LE(data + layout.timestamp_at);
  result.total = ReadU32LE(data + layout.total_at);
  result.index = ReadU32LE(data + layout.index_at);
  result.pos_num = pos_num;

  if (result.total == 0) {
    result.reason = MakeReason("total point count is zero");
    return false;
  }

  // Decode the points.
  std::vector<LidarPoint> pts;
  pts.reserve(pos_num);
  for (std::size_t i = 0; i < pos_num; ++i) {
    const std::uint8_t* p = data + layout.pos_at + i * kPointBytes;
    const double x = static_cast<double>(ReadI32LE(p)) *
                     config.point_position_scale;
    const double y = static_cast<double>(ReadI32LE(p + 4)) *
                     config.point_position_scale;
    const double z = static_cast<double>(ReadI32LE(p + 8)) *
                     config.point_position_scale;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      result.reason = MakeReason("point coordinate scaled to non-finite");
      return false;
    }
    LidarPoint pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    pt.rgba_channels[0] = ReadU32LE(p + 12);
    pt.rgba_channels[1] = ReadU32LE(p + 16);
    pt.rgba_channels[2] = ReadU32LE(p + 20);
    pt.rgba_channels[3] = ReadU32LE(p + 24);
    pt.rgba = pt.rgba_channels[0];
    pts.push_back(pt);
  }
  result.points = std::move(pts);
  result.ok = true;
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// Odometry parsing.
// ---------------------------------------------------------------------------

// Odometry header layouts. Packed: head@0, ts@2, x@10, tail@66 (68 bytes).
// Aligned: head@0, ts@8, x@16, tail@72 (80 bytes).
struct OdomLayout {
  std::size_t len;
  std::size_t timestamp_at;
  std::size_t x_at;
  std::size_t tail_at;
};

constexpr OdomLayout kPackedOdom = {68, 2, 10, 66};
constexpr OdomLayout kAlignedOdom = {80, 8, 16, 72};

bool ParseOdomWithLayout(const std::uint8_t* data, std::size_t len,
                         const OdomLayout& layout,
                         const LidarParseConfig& config,
                         OdometryPacketResult& result) {
  if (len != layout.len) {
    return false;
  }
  if (ReadU16LE(data) != kHeadMarker) {
    result.reason = MakeReason("head marker is not 0xAA55");
    return false;
  }
  if (ReadU16LE(data + layout.tail_at) != kTailMarker) {
    result.reason = MakeReason("tail marker is not 0xFF00");
    return false;
  }

  LidarOdometry odom;
  odom.timestamp_ns = ReadU64LE(data + layout.timestamp_at);

  // Translation: int64 * odom_position_scale, checked for finite range.
  const double sx = config.odom_position_scale;
  odom.x = static_cast<double>(ReadI64LE(data + layout.x_at)) * sx;
  odom.y = static_cast<double>(ReadI64LE(data + layout.x_at + 8)) * sx;
  odom.z = static_cast<double>(ReadI64LE(data + layout.x_at + 16)) * sx;
  if (!std::isfinite(odom.x) || !std::isfinite(odom.y) ||
      !std::isfinite(odom.z)) {
    result.reason = MakeReason("odometry position scaled to non-finite");
    return false;
  }

  if (!ValidQuaternionOrder(config.odom_quaternion_order)) {
    result.reason = MakeReason("odom_quaternion_order must be 'xyzw' or 'wxyz'");
    return false;
  }
  const double sq = config.odom_quaternion_scale;
  const double raw0 =
      static_cast<double>(ReadI64LE(data + layout.x_at + 24)) * sq;
  const double raw1 =
      static_cast<double>(ReadI64LE(data + layout.x_at + 32)) * sq;
  const double raw2 =
      static_cast<double>(ReadI64LE(data + layout.x_at + 40)) * sq;
  const double raw3 =
      static_cast<double>(ReadI64LE(data + layout.x_at + 48)) * sq;
  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
  if (config.odom_quaternion_order == "wxyz") {
    qw = raw0;
    qx = raw1;
    qy = raw2;
    qz = raw3;
  } else {  // "xyzw"
    qx = raw0;
    qy = raw1;
    qz = raw2;
    qw = raw3;
  }
  const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (!std::isfinite(norm) || norm == 0.0) {
    result.reason = MakeReason("odometry quaternion norm is zero");
    return false;
  }
  odom.qx = qx / norm;
  odom.qy = qy / norm;
  odom.qz = qz / norm;
  odom.qw = qw / norm;

  result.odometry = odom;
  result.ok = true;
  return true;
}

OdometryPacketResult ParseOdometryPacket(const std::uint8_t* data,
                                         std::size_t len,
                                         const LidarParseConfig& config) {
  OdometryPacketResult result;
  if (!ValidateScales(config)) {
    result.reason = MakeReason("invalid scale coefficients");
    return result;
  }
  // Reject a null/empty buffer before any marker read below dereferences it.
  if (data == nullptr || len < 2U) {
    result.reason = MakeReason("datagram too short");
    return result;
  }
  if (len < kPackedOdom.len && len < kAlignedOdom.len) {
    result.reason = MakeReason("datagram too short");
    return result;
  }
  if (ReadU16LE(data) != kHeadMarker) {
    result.reason = MakeReason("head marker is not 0xAA55");
    return result;
  }
  // Packed layout first, then aligned as a fallback.
  if (ParseOdomWithLayout(data, len, kPackedOdom, config, result)) {
    return result;
  }
  if (ParseOdomWithLayout(data, len, kAlignedOdom, config, result)) {
    return result;
  }
  if (result.reason.empty()) {
    result.reason = MakeReason("datagram length matches neither layout");
  }
  return result;
}

// ---------------------------------------------------------------------------
// Point-cloud parsing.
// ---------------------------------------------------------------------------

PointCloudPacketResult ParsePointCloudPacket(const std::uint8_t* data,
                                             std::size_t len,
                                             const LidarParseConfig& config) {
  PointCloudPacketResult result;
  if (!ValidateScales(config)) {
    result.reason = MakeReason("invalid scale coefficients");
    return result;
  }
  if (data == nullptr || len < 2U) {
    result.reason = MakeReason("datagram too short");
    return result;
  }
  if (ReadU16LE(data) != kHeadMarker) {
    result.reason = MakeReason("head marker is not 0xAA55");
    return result;
  }
  // Packed header first, then the type-aligned header as fallback.
  if (ParsePointCloudWithLayout(data, len, kPackedCloud, config, result)) {
    return result;
  }
  // Keep any specific reason from the packed attempt; overwrite only when
  // the aligned attempt gives a more specific one.
  const std::string packed_reason = result.reason;
  if (ParsePointCloudWithLayout(data, len, kAlignedCloud, config, result)) {
    return result;
  }
  result.reason = packed_reason.empty() ? result.reason : packed_reason;
  result.ok = false;
  return result;
}

// ---------------------------------------------------------------------------
// Frame assembler.
// ---------------------------------------------------------------------------

PointCloudFrameAssembler::PointCloudFrameAssembler(AssemblerConfig config)
    : config_(config) {
  if (config_.max_parallel_frames == 0) {
    config_.max_parallel_frames = 1;
  }
  slots_.resize(config_.max_parallel_frames);
}

PointCloudFrameAssembler::FrameSlot* PointCloudFrameAssembler::
    slot_for_timestamp(std::uint64_t timestamp_ns, bool create) {
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (slots_[i].active && slots_[i].timestamp_ns == timestamp_ns) {
      return &slots_[i];
    }
  }
  if (!create) {
    return nullptr;
  }

  // Find a free slot; otherwise the oldest incomplete frame is evicted
  // (capacity drop).
  std::size_t free_slot = slots_.size();
  std::size_t oldest_slot = slots_.size();
  auto oldest_time = std::chrono::steady_clock::time_point::max();
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (!slots_[i].active) {
      free_slot = i;
      break;
    }
    if (slots_[i].activated_at < oldest_time) {
      oldest_time = slots_[i].activated_at;
      oldest_slot = i;
    }
  }

  std::size_t chosen = free_slot;
  if (chosen == slots_.size()) {
    chosen = oldest_slot;
    // The evicted frame is dropped to make room (capacity drop); its
    // accepted packets were already counted.
    finalize_slot(&slots_[chosen], FinalizeReason::Published);
    ++stats_.frames_dropped_capacity;
  }

  slots_[chosen].active = true;
  slots_[chosen].timestamp_ns = timestamp_ns;
  slots_[chosen].total = 0;
  slots_[chosen].packets.clear();
  slots_[chosen].unique_indices = 0;
  slots_[chosen].sum_points = 0;
  slots_[chosen].activated_at = std::chrono::steady_clock::now();
  return &slots_[chosen];
}

std::optional<PointCloudFrame> PointCloudFrameAssembler::add_packet(
    const PointCloudPacketResult& packet) {
  if (!packet.ok) {
    // Callers are expected to feed only parsed packets; a non-ok packet
    // still counts as a reject.
    ++stats_.packets_rejected;
    return std::nullopt;
  }
  // The frame must declare a positive total within the per-frame point cap.
  if (packet.total == 0 || packet.total > config_.max_points_per_frame) {
    ++stats_.packets_rejected;
    return std::nullopt;
  }
  // Per the manual `index` is the packet sequence number within the frame;
  // storage is keyed by it, so bound it by max_packets_per_frame to keep
  // memory bounded. A packet whose index exceeds the cap is rejected (the
  // 1-based ceiling; the stricter 0-based ceiling is index >= cap and that
  // value remains bounded but can only ever be valid for a 1-based frame).
  if (packet.index > config_.max_packets_per_frame) {
    ++stats_.packets_rejected;
    return std::nullopt;
  }

  FrameSlot* slot = slot_for_timestamp(packet.timestamp_ns, true);
  if (slot == nullptr) {
    ++stats_.packets_rejected;
    return std::nullopt;
  }

  // Frame-wide total inconsistency for the same timestamp is a conflicting
  // frame: drop it.
  if (slot->unique_indices > 0 && slot->total != packet.total) {
    finalize_slot(slot, FinalizeReason::Conflict);
    return std::nullopt;
  }
  if (slot->unique_indices == 0) {
    slot->total = packet.total;
  }

  // Duplicate index: identical content is a harmless duplicate (ignored);
  // differing content is a conflict that drops the whole frame.
  if (packet.index < slot->packets.size() &&
      !slot->packets[packet.index].empty()) {
    if (slot->packets[packet.index] == packet.points) {
      ++stats_.packets_duplicated;
      return std::nullopt;
    }
    finalize_slot(slot, FinalizeReason::Conflict);
    return std::nullopt;
  }

  // Accumulated point-count budget: accepting this packet must not drive the
  // frame's point total past its declared `total` or past the absolute
  // per-frame cap. Either is a conflicting (inconsistent) frame, dropped.
  const std::size_t points_here = packet.points.size();
  if (slot->sum_points + points_here > slot->total ||
      slot->sum_points + points_here > config_.max_points_per_frame) {
    finalize_slot(slot, FinalizeReason::Conflict);
    return std::nullopt;
  }

  // Store keyed by the raw packet index (slot 0 is simply unused for a
  // 1-based frame). The index set bounded by max_packets_per_frame keeps the
  // slot vector bounded.
  if (slot->packets.size() <= packet.index) {
    slot->packets.resize(static_cast<std::size_t>(packet.index) + 1);
  }
  slot->packets[packet.index] = packet.points;
  ++slot->unique_indices;
  slot->sum_points += points_here;
  ++stats_.packets_accepted;

  // Completion check. The manually-defined semantics require TWO conditions:
  //  (1) the observed packet-index set covers contiguously starting from the
  //      base (index 0 present => 0-based; index 0 absent but index 1 present
  //      => 1-based), leaving no gap up to the highest observed index;
  //  (2) the points across those packets sum exactly to the declared total.
  // The base is never assumed - index 0's presence proves it.
  bool complete = false;
  if (slot->sum_points == slot->total && slot->unique_indices > 0) {
    // Highest observed index.
    std::size_t highest = slot->packets.size();
    while (highest > 0 && slot->packets[highest - 1].empty()) {
      --highest;
    }
    const bool base0 = highest > 0 && !slot->packets[0].empty();
    const bool base1 = highest > 1 && slot->packets[0].empty() &&
                       !slot->packets[1].empty();
    if (base0 || base1) {
      const std::size_t base = base0 ? 0U : 1U;
      bool contiguous = true;
      for (std::size_t i = base; i < highest; ++i) {
        if (slot->packets[i].empty()) {
          contiguous = false;
          break;
        }
      }
      complete = contiguous;
    }
  }

  if (!complete) {
    return std::nullopt;
  }

  // Emit the frame with points in packet-index order (storage is already
  // keyed by raw index, so iterating slots 0..size-1 yields index order for
  // either base).
  PointCloudFrame frame;
  frame.timestamp_ns = slot->timestamp_ns;
  frame.total_points = slot->total;
  frame.points.reserve(slot->total);
  for (std::size_t i = 0; i < slot->packets.size(); ++i) {
    const auto& pts = slot->packets[i];
    frame.points.insert(frame.points.end(), pts.begin(), pts.end());
  }
  if (frame.points.size() != slot->total) {
    // Defensive: the completeness check guarantees the count; a mismatch
    // here means a conflicting frame slipped through.
    finalize_slot(slot, FinalizeReason::Conflict);
    return std::nullopt;
  }

  ++stats_.frames_published;
  finalize_slot(slot, FinalizeReason::Published);
  return frame;
}

// Clears an active slot and bumps the appropriate drop/finalize counter.
void PointCloudFrameAssembler::finalize_slot(FrameSlot* slot,
                                             FinalizeReason reason) {
  if (slot == nullptr || !slot->active) {
    return;
  }
  slot->active = false;
  slot->packets.clear();
  slot->unique_indices = 0;
  slot->sum_points = 0;
  switch (reason) {
    case FinalizeReason::Timeout:
      ++stats_.frames_dropped_timeout;
      break;
    case FinalizeReason::Conflict:
      ++stats_.frames_dropped_conflict;
      break;
    case FinalizeReason::Published:
      break;  // counts handled by the caller
  }
}

void PointCloudFrameAssembler::expire(
    std::chrono::steady_clock::time_point now) {
  for (std::size_t i = 0; i < slots_.size(); ++i) {
    if (!slots_[i].active) {
      continue;
    }
    const auto age = now - slots_[i].activated_at;
    if (age >= config_.frame_timeout) {
      finalize_slot(&slots_[i], FinalizeReason::Timeout);
    }
  }
}

std::size_t PointCloudFrameAssembler::in_flight_count() const {
  std::size_t n = 0;
  for (const auto& slot : slots_) {
    if (slot.active) {
      ++n;
    }
  }
  return n;
}

PointCloudFrameAssembler::Stats PointCloudFrameAssembler::stats() const {
  return stats_;
}

// ---------------------------------------------------------------------------
// UDP datagram source.
// ---------------------------------------------------------------------------

UdpLidarDatagramSource::UdpLidarDatagramSource(
    std::uint16_t port, std::string bind_ip, std::string expected_source_ip)
    : fd_(-1), expected_source_ip_(std::move(expected_source_ip)) {
  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    return;
  }
  int reuse = 1;
  setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, bind_ip.c_str(), &addr.sin_addr) != 1) {
    close(fd_);
    fd_ = -1;
    return;
  }
  if (bind(fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
    close(fd_);
    fd_ = -1;
    return;
  }
}

UdpLidarDatagramSource::~UdpLidarDatagramSource() {
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool UdpLidarDatagramSource::valid() const { return fd_ >= 0; }

bool UdpLidarDatagramSource::receive(std::vector<std::uint8_t>& out,
                                     std::chrono::milliseconds timeout) {
  if (fd_ < 0) {
    ++stats_.errors;
    return false;
  }

  // poll(2) for the receive timeout (POSIX; no ROS dependency).
  pollfd pfd{fd_, POLLIN, 0};
  int timeout_ms = static_cast<int>(timeout.count());
  if (timeout_ms < 0) {
    timeout_ms = 0;
  }
  int pr = poll(&pfd, 1, timeout_ms);
  if (pr == 0) {
    ++stats_.timeouts;
    return false;
  }
  if (pr < 0) {
    ++stats_.errors;
    return false;
  }

  sockaddr_in from{};
  socklen_t from_len = sizeof(from);
  std::uint8_t buf[65536];
  const ssize_t n = recvfrom(fd_, buf, sizeof(buf), 0,
                             reinterpret_cast<sockaddr*>(&from), &from_len);
  if (n < 0) {
    ++stats_.errors;
    return false;
  }

  // Optional source-address filtering.
  if (!expected_source_ip_.empty()) {
    char ip[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
    if (expected_source_ip_ != ip) {
      ++stats_.dropped_source;
      return false;
    }
  }

  ++stats_.received;
  out.insert(out.end(), buf, buf + static_cast<std::size_t>(n));
  return true;
}

UdpLidarDatagramSource::Stats UdpLidarDatagramSource::stats() const {
  return stats_;
}

// ---------------------------------------------------------------------------
// Receiver.
// ---------------------------------------------------------------------------

// Each receiver keeps its own impl (single socket, single thread); sharing
// the impl across receivers is not needed. `shared_ptr` here only so the
// receive thread can own the same impl the receiver methods touch after
// stop() joined it, keeping reads on objects the thread no longer touches.
struct LidarStreamReceiver::Impl {
  Impl(std::unique_ptr<ILidarDatagramSource> s, LidarStreamKind k,
       LidarParseConfig c, AssemblerConfig ac, LidarSink snk,
       std::function<void(const std::string&)> warn)
      : source(std::move(s)),
        kind(k),
        config(c),
        assembler(ac),
        sink(std::move(snk)),
        on_parse_warning(std::move(warn)) {}

  std::unique_ptr<ILidarDatagramSource> source;
  LidarStreamKind kind;
  LidarParseConfig config;
  PointCloudFrameAssembler assembler;

  // Guards the socket receive loop against concurrent stop/start and
  // protects the run flag and parse-warning throttle state. Callbacks are
  // always invoked OUTSIDE this lock.
  std::mutex mutex;
  std::condition_variable cv;
  bool stop_requested{false};
  bool running{false};
  bool start_called{false};
  std::thread thread;
  // The receive thread's id, recorded at start() under `mutex` and read in
  // stop() under `mutex` so a stop() invoked from inside a callback can tell
  // it is already on the receive thread and must not join itself.
  std::thread::id thread_id;
  // Serializes the stop()+join critical section across external threads so
  // the thread is joined exactly once even under concurrent stop() calls.
  std::mutex stop_mutex;

  // Parse-warning throttle (at most one per second).
  std::chrono::steady_clock::time_point last_warning{};
  std::mutex warn_mutex;

  // Callbacks with its own synchronization since they are invoked without
  // the loop mutex held.
  LidarSink sink;
  std::function<void(const std::string&)> on_parse_warning;

  // 1 Hz throttle guarding emission of a parse warning.
  bool should_emit_warning() {
    std::lock_guard<std::mutex> lock(warn_mutex);
    const auto now = std::chrono::steady_clock::now();
    if (now - last_warning < std::chrono::seconds(1)) {
      return false;
    }
    last_warning = now;
    return true;
  }

  // Run loops (called on the receive thread, never under `mutex`).
  void loop();
  void loop_point_cloud();
  void loop_odometry();

  // Invoke a sink callback, swallowing any exception so a throwing callback
  // never kills the receive thread and never crosses a lock.
  template <typename F>
  static void InvokeSafe(F&& fn) {
    try {
      fn();
    } catch (...) {
      // Intentionally swallowed.
    }
  }
};

void LidarStreamReceiver::Impl::loop_odometry() {
  while (true) {
    {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [this] { return stop_requested || running; });
      if (stop_requested) {
        return;
      }
    }
    // One receive with a 50ms timeout keeps stop() latency bounded.
    std::vector<std::uint8_t> buffer;
    bool got = false;
    {
      got =
          source->receive(buffer, std::chrono::milliseconds(50));
    }
    if (!got) {
      continue;
    }
    const OdometryPacketResult parsed =
        ParseOdometryPacket(buffer.data(), buffer.size(), config);
    if (!parsed.ok) {
      if (on_parse_warning && should_emit_warning()) {
        auto warn = parsed.reason;
        InvokeSafe([&] { on_parse_warning(warn); });
      }
      continue;
    }
    if (sink.on_odometry) {
      auto odom = parsed.odometry;
      InvokeSafe([&] { sink.on_odometry(odom); });
    }
  }
}

void LidarStreamReceiver::Impl::loop_point_cloud() {
  while (true) {
    {
      std::unique_lock<std::mutex> lock(mutex);
      cv.wait(lock, [this] { return stop_requested || running; });
      if (stop_requested) {
        return;
      }
    }
    std::vector<std::uint8_t> buffer;
    bool got = source->receive(buffer, std::chrono::milliseconds(50));
    if (got) {
      const PointCloudPacketResult parsed =
          ParsePointCloudPacket(buffer.data(), buffer.size(), config);
      if (parsed.ok) {
        auto frame = assembler.add_packet(parsed);
        if (frame && sink.on_frame) {
          InvokeSafe([&] { sink.on_frame(*frame); });
        }
      } else {
        if (on_parse_warning && should_emit_warning()) {
          auto warn = parsed.reason;
          InvokeSafe([&] { on_parse_warning(warn); });
        }
      }
    }
    // Drive the assembler's steady-clock expiry on every loop iteration.
    assembler.expire(std::chrono::steady_clock::now());
  }
}

void LidarStreamReceiver::Impl::loop() {
  if (kind == LidarStreamKind::Odometry) {
    loop_odometry();
  } else {
    loop_point_cloud();
  }
}

LidarStreamReceiver::LidarStreamReceiver(
    std::unique_ptr<ILidarDatagramSource> source, LidarStreamKind kind,
    LidarParseConfig config, AssemblerConfig assembler_config, LidarSink sink,
    std::function<void(const std::string&)> on_parse_warning)
    : impl_(std::make_shared<Impl>(std::move(source), kind, config,
                                   assembler_config, std::move(sink),
                                   std::move(on_parse_warning))),
      kind_(kind) {}

LidarStreamReceiver::~LidarStreamReceiver() { stop(); }

void LidarStreamReceiver::start() {
  std::unique_lock<std::mutex> lock(impl_->mutex);
  if (impl_->stop_requested || impl_->start_called) {
    return;
  }
  impl_->start_called = true;
  impl_->running = true;
  impl_->cv.notify_all();
  impl_->thread = std::thread([impl = impl_] { impl->loop(); });
  impl_->thread_id = impl_->thread.get_id();
}

void LidarStreamReceiver::stop() noexcept {
  std::shared_ptr<Impl> impl = impl_;
  // 1. Request the loop to stop and wake it. Making this the first step on
  //    every caller means a stop() invoked from inside a callback still gets
  //    the loop to exit even though it cannot join the thread it is running
  //    on (which would be std::terminate).
  {
    std::lock_guard<std::mutex> lock(impl->mutex);
    impl->stop_requested = true;
    impl->running = false;
    impl->cv.notify_all();
    // 2. If this thread is the receive thread itself, do not join it: return
    //    and let the loop observe stop_requested and unwind. The read is
    //    under `mutex` so start()'s write is ordered.
    if (std::this_thread::get_id() == impl->thread_id) {
      return;
    }
  }
  // 3. External threads: serialize the join so the receive thread is joined
  //    exactly once. join() blocks here until the (now stop_requested) loop
  //    returns.
  std::lock_guard<std::mutex> lock(impl->stop_mutex);
  if (impl->thread.joinable()) {
    impl->thread.join();
  }
}

bool LidarStreamReceiver::running() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->running && !impl_->stop_requested;
}

PointCloudFrameAssembler::Stats LidarStreamReceiver::assembler_stats() const {
  return impl_->assembler.stats();
}

}  // namespace hypertron_ros2_bridge
