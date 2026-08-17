#pragma once

// Pure C++ SW01 LiDAR stream library: parsing and bounded frame reassembly
// for the AstrallPosCloudData (UDP 6100 point cloud) and
// AstrallOdometryData (UDP 6101 odometry) datagrams. This header has no
// vendor SDK, no ROS, and no boost dependency - only the standard library -
// so the library builds and is fully testable in the pure CMake
// configuration (BUILD_ROS2_BRIDGE=OFF).

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace hypertron_ros2_bridge {

// ---------------------------------------------------------------------------
// Payload value types.
// ---------------------------------------------------------------------------

// One 28-byte point from AstrallPosCloudData. The raw r/g/b/a uint32_t
// values keep the vendor byte order; this library never guesses a channel
// order (the upper layer decides how to interpret them). All four uint32_t
// channels are preserved so no bytes are discarded as padding.
struct LidarPoint {
  float x{};
  float y{};
  float z{};
  // First RGBA channel, kept for backward compatibility / PointXYZRGBA.
  std::uint32_t rgba{};
  // All four raw 32-bit RGBA channels from the 28-byte point record.
  std::array<std::uint32_t, 4> rgba_channels{};

  friend bool operator==(const LidarPoint& a, const LidarPoint& b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z &&
           a.rgba == b.rgba && a.rgba_channels == b.rgba_channels;
  }
};

// A complete point-cloud frame emitted by the assembler, with points ordered
// by packet index.
struct PointCloudFrame {
  std::uint64_t timestamp_ns{};
  std::uint64_t total_points{};
  std::vector<LidarPoint> points;
};

// Normalized odometry from one AstrallOdometryData packet.
struct LidarOdometry {
  std::uint64_t timestamp_ns{};
  double x{};
  double y{};
  double z{};
  double qx{};
  double qy{};
  double qz{};
  double qw{};
};

// ---------------------------------------------------------------------------
// Parse configuration and results.
// ---------------------------------------------------------------------------

// Scaling applied to the fixed-point coordinates read from the datagrams.
// Every coefficient is validated at parse time (finite, strictly positive,
// magnitude bounded), so misuse is rejected with a readable reason instead
// of silently producing garbage. These are the documented-allocation values;
// whether they match real hardware is a "verified" concern owned by the
// upper node, never by this library: this library only provides the
// coefficients and the validation.
struct LidarParseConfig {
  // Meter factor for the int32 point coordinates (×point_position_scale).
  double point_position_scale{1e-3};
  // Meter factor for the int64 odometry x/y/z (×odom_position_scale).
  double odom_position_scale{1e-6};
  // Scale applied to the int64 odometry quaternion components before the
  // resulting vector is normalized to unit length.
  double odom_quaternion_scale{1.0};
  // Order of the four int64 quaternion components in the UDP 6101 packet.
  // "xyzw" (default) reads them as qx,qy,qz,qw; "wxyz" reads w,x,y,z.
  std::string odom_quaternion_order{"xyzw"};
};

// Uniform parse outcome for a whole datagram. `ok` is true only when the
// whole datagram parsed cleanly; `reason` is always a readable message. The
// "bool ok + message" shape mirrors the Result convention used elsewhere in
// the repository, extended with the decoded payload fields described below.

// Result of parsing one AstrallPosCloudData packet. `points` holds the
// posNum decoded points (already scale-applied).
struct PointCloudPacketResult {
  bool ok{false};
  std::string reason;
  std::uint64_t timestamp_ns{};
  std::uint64_t total{};
  std::uint32_t index{};
  std::uint16_t pos_num{};
  std::vector<LidarPoint> points;
};

// Result of parsing one AstrallOdometryData packet.
struct OdometryPacketResult {
  bool ok{false};
  std::string reason;
  LidarOdometry odometry;
};

// Parses one AstrallPosCloudData datagram. Stateless and thread-safe
// (pure function over the byte buffer). Accepts both vendor layouts - the
// naturally-packed header AND the type-aligned header, each with either a
// dynamic tail (exactly posNum points) or a fixed 50-point tail (posNum
// points extracted, remainder ignored) - by validating size plus the
// 0xAA55/0xFF00 head/tail markers.
PointCloudPacketResult ParsePointCloudPacket(
    const std::uint8_t* data, std::size_t len,
    const LidarParseConfig& config);

// Parses one AstrallOdometryData datagram. Tries the packed 68-byte header
// first, then the type-aligned 80-byte header (packed first, aligned as
// fallback, matching the legacy parser behavior).
OdometryPacketResult ParseOdometryPacket(
    const std::uint8_t* data, std::size_t len,
    const LidarParseConfig& config);

// ---------------------------------------------------------------------------
// Frame reassembly.
// ---------------------------------------------------------------------------

struct AssemblerConfig {
  // Maximum frames being assembled concurrently; a new frame beyond this
  // count evicts the oldest incomplete one.
  std::size_t max_parallel_frames{4};
  // A frame incomplete after this long is dropped as a timeout.
  std::chrono::milliseconds frame_timeout{300};
  // A frame whose declared total exceeds this is rejected outright.
  std::size_t max_points_per_frame{2000000};
  // Upper bound on the number of packets a single frame may span. Per the
  // SW01 manual, each packet carries `index` = its packet sequence number
  // (within the frame) and `total` = the frame's total point count; a frame
  // completes only when its observed packet-index set covers contiguously
  // from the base (0 or 1) AND the points across those packets sum to
  // `total`. Storage is keyed by the raw packet index and this cap makes it
  // bounded: a packet whose index exceeds this cap (1-based: index > cap;
  // the 0-based analogue is index >= cap) is rejected outright, so a sparse
  // or hostile index sequence cannot allocate unbounded memory.
  //
  // The default matches max_points_per_frame=2,000,000 / 50 points per
  // packet = 40,000 packets, so a full frame is not rejected by the packet
  // cap.
  std::size_t max_packets_per_frame{40000};
};

// Reassembles a single point-cloud frame from its UDP packet sequence
// (each frame is split across several indexed packets). Per the SW01 manual
// `index` is the packet's sequence number within the current frame and
// `total` is the frame's total point count. The index base (0-based or
// 1-based) is deliberately not assumed: a frame publishes only when its
// observed index set covers contiguously from the base (0 or 1) AND the
// points across those packets sum to `total`, so the observed indices prove
// their own base and the point count proves the frame is whole. Packets whose
// declared total exceeds max_points_per_frame, whose index exceeds
// max_packets_per_frame, or that would drive the slot's accumulated point
// count past `total` or max_points_per_frame are rejected (the frame dropped
// as a conflict in the last case).
//
// Single-threaded use only. Thread coordination is the responsibility of the
// upper layer (see LidarStreamReceiver). No method is internally locked.
class PointCloudFrameAssembler {
 public:
  explicit PointCloudFrameAssembler(AssemblerConfig config = {});
  ~PointCloudFrameAssembler() = default;
  PointCloudFrameAssembler(const PointCloudFrameAssembler&) = delete;
  PointCloudFrameAssembler& operator=(const PointCloudFrameAssembler&) =
      delete;

  // Feeds one successfully parsed packet into its frame. Returns the
  // completed frame (points in index order) the moment that frame becomes
  // complete; returns std::nullopt when the frame is still incomplete or
  // the packet was rejected/duplicated. A rejected or consumed-invalid
  // packet still counts in `stats()`.
  std::optional<PointCloudFrame> add_packet(
      const PointCloudPacketResult& packet);

  // Drops every incomplete frame whose age exceeds frame_timeout, counting
  // each as a timeout drop. Call periodically with a steady_clock time
  // point; the owning receiver drives this from its receive loop.
  void expire(std::chrono::steady_clock::time_point now);

  std::size_t in_flight_count() const;

  struct Stats {
    std::uint64_t packets_accepted{};
    std::uint64_t packets_duplicated{};
    std::uint64_t packets_rejected{};
    std::uint64_t frames_published{};
    std::uint64_t frames_dropped_timeout{};
    std::uint64_t frames_dropped_conflict{};
    std::uint64_t frames_dropped_capacity{};
  };
  Stats stats() const;

 private:
  enum class FinalizeReason { Published, Timeout, Conflict };

  struct FrameSlot {
    // Set when the frame was requested by a packet; cleared on publish or
    // drop. Empty while the slot is free.
    bool active{false};
    std::uint64_t timestamp_ns{};
    std::uint64_t total{};  // declared total for the slot version
    // storage keyed by raw packet index -> point data. An empty inner
    // vector means that index is not (yet) observed. Slot 0 is unused for a
    // 1-based frame.
    std::vector<std::vector<LidarPoint>> packets;
    std::size_t unique_indices{};
    // Sum of points across the uniquely observed packets for this slot. A
    // frame completes only when this equals the declared total (and the
    // observed index set is contiguous from the base).
    std::size_t sum_points{};
    // When this slot was (re)activated; used for expire().
    std::chrono::steady_clock::time_point activated_at{};
  };

  // Finds the slot holding the given timestamp (optionally creating it when
  // create is true and capacity allows), or returns nullptr. Evicts the
  // oldest incomplete frame when the capacity limit forces it.
  FrameSlot* slot_for_timestamp(std::uint64_t timestamp_ns, bool create);

  // Clears an active slot and bumps the appropriate drop/finalize counter.
  void finalize_slot(FrameSlot* slot, FinalizeReason reason);

  std::vector<FrameSlot> slots_;
  AssemblerConfig config_;
  Stats stats_;
};

// ---------------------------------------------------------------------------
// Datagram transport.
// ---------------------------------------------------------------------------

// Abstract transport over which one LiDAR datagram is received. The library
// ships a UDP implementation; tests inject programmatic fakes.
class ILidarDatagramSource {
 public:
  virtual ~ILidarDatagramSource() = default;
  // Reads one datagram into `out` (appends to any existing content) or
  // returns false when no data arrives within `timeout`.
  virtual bool receive(std::vector<std::uint8_t>& out,
                       std::chrono::milliseconds timeout) = 0;
};

// UDP datagram source bound to a fixed port. Binds with SO_REUSEADDR, and
// when expected_source_ip is non-empty drops (and counts) any packet whose
// recvfrom source does not match - a dropped packet is not treated as a
// timeout. The recv timeout is implemented with poll(2) (POSIX; no ROS).
class UdpLidarDatagramSource final : public ILidarDatagramSource {
 public:
  UdpLidarDatagramSource(std::uint16_t port,
                         std::string bind_ip = "0.0.0.0",
                         std::string expected_source_ip = "");
  ~UdpLidarDatagramSource() override;
  UdpLidarDatagramSource(const UdpLidarDatagramSource&) = delete;
  UdpLidarDatagramSource& operator=(const UdpLidarDatagramSource&) = delete;

  bool receive(std::vector<std::uint8_t>& out,
               std::chrono::milliseconds timeout) override;

  // True when the socket underlying the source is open. Used by tests and
  // by ownership transfer; the receiver uses this only for diagnostics.
  bool valid() const;

  struct Stats {
    std::uint64_t received{};
    std::uint64_t dropped_source{};
    std::uint64_t timeouts{};
    std::uint64_t errors{};
  };
  Stats stats() const;

 private:
  int fd_;
  std::string expected_source_ip_;
  mutable Stats stats_;
};

// Callback sink for complete frames and odometry. The receiver guarantees
// callbacks never run inside a lock and never let a throwing callback escape
// the receive thread (the receiver catches everything).
struct LidarSink {
  std::function<void(const PointCloudFrame&)> on_frame;
  std::function<void(const LidarOdometry&)> on_odometry;
};

// ---------------------------------------------------------------------------
// Receiver.
// ---------------------------------------------------------------------------

// Owns the receive thread for a single LiDAR stream on a single socket.
class LidarStreamReceiver {
 public:
  enum class LidarStreamKind { PointCloud, Odometry };

  // Takes ownership of the source. For point-cloud streams the assembler
  // reassembles complete frames from packets; for odometry streams every
  // packet is delivered directly. `on_parse_warning` is throttled to at
  // most one call per second and reports a readable parse reason; it stays
  // optional.
  LidarStreamReceiver(std::unique_ptr<ILidarDatagramSource> source,
                      LidarStreamKind kind, LidarParseConfig config,
                      AssemblerConfig assembler_config, LidarSink sink,
                      std::function<void(const std::string&)>
                          on_parse_warning = {});
  ~LidarStreamReceiver();

  LidarStreamReceiver(const LidarStreamReceiver&) = delete;
  LidarStreamReceiver& operator=(const LidarStreamReceiver&) = delete;

  // Starts the receive thread and returns immediately. A no-op when already
  // running.
  void start();
  // Requests shutdown, wakes the receive loop, and joins the thread.
  // Idempotent and thread-safe. "Requesting a stop" and "joining the thread"
  // are split so a stop() from inside a callback (i.e. on the receive thread
  // itself) only requests the shutdown and returns without joining (joining
  // the running thread would be std::terminate); an external stop() then
  // performs the single join. The destructor always runs on a non-receive
  // thread and always joins. Concurrent stop() calls serialize on an internal
  // mutex and join exactly once.
  void stop() noexcept;
  bool running() const;

  PointCloudFrameAssembler::Stats assembler_stats() const;
  const LidarStreamKind& kind() const { return kind_; }

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  LidarStreamKind kind_;
};

}  // namespace hypertron_ros2_bridge
