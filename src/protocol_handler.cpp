#include "hypertron_ros2_bridge/protocol_handler.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

namespace hypertron_ros2_bridge {
namespace {

constexpr std::array<std::uint8_t, 4> kMagic{{'H', 'T', 'B', 'R'}};
constexpr std::size_t kHeaderSize = 28U;
constexpr std::uint32_t kKnownCapabilities =
    CapabilityImu | CapabilitySport | CapabilityOdometry | CapabilityCamera |
    CapabilityJoint | CapabilitySystemState;

void validate_capabilities(std::uint32_t capabilities) {
  if ((capabilities & ~kKnownCapabilities) != 0U) {
    throw ProtocolError("capability bitset contains reserved bits");
  }
}

class Writer {
 public:
  void u8(std::uint8_t value) { bytes_.push_back(value); }
  void u16(std::uint16_t value) {
    bytes_.push_back(static_cast<std::uint8_t>(value >> 8U));
    bytes_.push_back(static_cast<std::uint8_t>(value));
  }
  void u32(std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) {
      bytes_.push_back(static_cast<std::uint8_t>(value >> shift));
    }
  }
  void u64(std::uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8) {
      bytes_.push_back(static_cast<std::uint8_t>(value >> shift));
    }
  }
  void f32(float value) {
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "32-bit IEEE-754 float required");
    std::uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    u32(bits);
  }
  void f64(double value) {
    static_assert(sizeof(double) == sizeof(std::uint64_t),
                  "64-bit IEEE-754 double required");
    std::uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    u64(bits);
  }
  void raw(const std::vector<std::uint8_t>& bytes) {
    bytes_.insert(bytes_.end(), bytes.begin(), bytes.end());
  }
  void text(const std::string& value) {
    if (value.size() > std::numeric_limits<std::uint16_t>::max()) {
      throw ProtocolError("text payload is too long");
    }
    u16(static_cast<std::uint16_t>(value.size()));
    bytes_.insert(bytes_.end(), value.begin(), value.end());
  }
  std::vector<std::uint8_t> finish() { return std::move(bytes_); }

 private:
  std::vector<std::uint8_t> bytes_;
};

class Reader {
 public:
  explicit Reader(const std::vector<std::uint8_t>& bytes) : bytes_(bytes) {}

  std::uint8_t u8() {
    require(1);
    return bytes_[offset_++];
  }
  std::uint16_t u16() {
    require(2);
    const auto value = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(bytes_[offset_]) << 8U) |
        bytes_[offset_ + 1U]);
    offset_ += 2U;
    return value;
  }
  std::uint32_t u32() {
    require(4);
    std::uint32_t value{};
    for (int i = 0; i < 4; ++i) {
      value = (value << 8U) | bytes_[offset_++];
    }
    return value;
  }
  std::uint64_t u64() {
    require(8);
    std::uint64_t value{};
    for (int i = 0; i < 8; ++i) {
      value = (value << 8U) | bytes_[offset_++];
    }
    return value;
  }
  float f32() {
    const auto bits = u32();
    float value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }
  double f64() {
    const auto bits = u64();
    double value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }
  std::string text() {
    const auto size = u16();
    require(size);
    std::string value(bytes_.begin() + static_cast<std::ptrdiff_t>(offset_),
                      bytes_.begin() +
                          static_cast<std::ptrdiff_t>(offset_ + size));
    offset_ += size;
    return value;
  }
  std::vector<std::uint8_t> rest() {
    std::vector<std::uint8_t> value(
        bytes_.begin() + static_cast<std::ptrdiff_t>(offset_), bytes_.end());
    offset_ = bytes_.size();
    return value;
  }
  void finish() const {
    if (offset_ != bytes_.size()) {
      throw ProtocolError("payload contains trailing bytes");
    }
  }

 private:
  void require(std::size_t count) const {
    if (count > bytes_.size() - offset_) {
      throw ProtocolError("payload is truncated");
    }
  }
  const std::vector<std::uint8_t>& bytes_;
  std::size_t offset_{0};
};

std::uint16_t read_u16_be(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(p[0]) << 8U) | p[1]);
}

std::uint32_t read_u32_be(const std::uint8_t* p) {
  std::uint32_t value{};
  for (int i = 0; i < 4; ++i) {
    value = (value << 8U) | p[i];
  }
  return value;
}

std::uint64_t read_u64_be(const std::uint8_t* p) {
  std::uint64_t value{};
  for (int i = 0; i < 8; ++i) {
    value = (value << 8U) | p[i];
  }
  return value;
}

std::uint16_t read_u16_le(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(
      p[0] | (static_cast<std::uint16_t>(p[1]) << 8U));
}

std::uint32_t read_u32_le(const std::uint8_t* p) {
  std::uint32_t value{};
  for (int i = 3; i >= 0; --i) {
    value = (value << 8U) | p[i];
  }
  return value;
}

std::uint64_t read_u64_le(const std::uint8_t* p) {
  std::uint64_t value{};
  for (int i = 7; i >= 0; --i) {
    value = (value << 8U) | p[i];
  }
  return value;
}

std::int64_t read_i64_le(const std::uint8_t* p) {
  return static_cast<std::int64_t>(read_u64_le(p));
}

std::int32_t read_i32_le(const std::uint8_t* p) {
  return static_cast<std::int32_t>(read_u32_le(p));
}

void write_u32_be(std::uint8_t* p, std::uint32_t value) {
  p[0] = static_cast<std::uint8_t>(value >> 24U);
  p[1] = static_cast<std::uint8_t>(value >> 16U);
  p[2] = static_cast<std::uint8_t>(value >> 8U);
  p[3] = static_cast<std::uint8_t>(value);
}

bool known_type(std::uint8_t value) {
  switch (static_cast<MessageType>(value)) {
    case MessageType::Hello:
    case MessageType::HelloAck:
    case MessageType::Ping:
    case MessageType::Pong:
    case MessageType::CmdVelocity:
    case MessageType::CmdMode:
    case MessageType::CmdEstop:
    case MessageType::CmdJoint:
    case MessageType::Imu:
    case MessageType::Sport:
    case MessageType::Odometry:
    case MessageType::RobotState:
    case MessageType::CameraH264:
    case MessageType::Ack:
    case MessageType::Error:
      return true;
  }
  return false;
}

template <std::size_t N>
void write_f32_array(Writer& writer, const std::array<float, N>& values) {
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      throw ProtocolError("payload contains a non-finite float");
    }
    writer.f32(value);
  }
}

template <std::size_t N>
std::array<float, N> read_f32_array(Reader& reader) {
  std::array<float, N> values{};
  for (auto& value : values) {
    value = reader.f32();
    if (!std::isfinite(value)) {
      throw ProtocolError("payload contains a non-finite float");
    }
  }
  return values;
}

template <std::size_t N>
void write_f64_array(Writer& writer, const std::array<double, N>& values) {
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      throw ProtocolError("payload contains a non-finite double");
    }
    writer.f64(value);
  }
}

template <std::size_t N>
std::array<double, N> read_f64_array(Reader& reader) {
  std::array<double, N> values{};
  for (auto& value : values) {
    value = reader.f64();
    if (!std::isfinite(value)) {
      throw ProtocolError("payload contains a non-finite double");
    }
  }
  return values;
}

}  // namespace

bool Frame::operator==(const Frame& o) const {
  return type == o.type && sequence == o.sequence &&
         monotonic_time_ns == o.monotonic_time_ns && payload == o.payload;
}

ProtocolHandler::ProtocolHandler(std::uint32_t max_payload)
    : max_payload_(max_payload) {
  if (max_payload_ == 0U) {
    throw std::invalid_argument("max payload must be greater than zero");
  }
}

std::vector<std::uint8_t> ProtocolHandler::encode(const Frame& frame) {
  if (frame.payload.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw ProtocolError("frame payload is too large");
  }
  std::vector<std::uint8_t> bytes(kHeaderSize, 0U);
  std::copy(kMagic.begin(), kMagic.end(), bytes.begin());
  bytes[4] = kBridgeProtocolVersion;
  bytes[5] = static_cast<std::uint8_t>(frame.type);
  write_u32_be(bytes.data() + 8U, frame.sequence);
  write_u32_be(bytes.data() + 12U,
               static_cast<std::uint32_t>(frame.payload.size()));
  for (int i = 0; i < 8; ++i) {
    bytes[16U + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(
        frame.monotonic_time_ns >> (56U - static_cast<unsigned>(i) * 8U));
  }
  bytes.insert(bytes.end(), frame.payload.begin(), frame.payload.end());
  write_u32_be(bytes.data() + 24U, crc32_iso_hdlc(bytes));
  return bytes;
}

std::vector<Frame> ProtocolHandler::feed(const std::uint8_t* bytes,
                                         std::size_t size) {
  if (size != 0U && bytes == nullptr) {
    throw std::invalid_argument("null input with non-zero size");
  }
  if (size != 0U) {
    buffer_.insert(buffer_.end(), bytes, bytes + size);
  }
  std::vector<Frame> frames;
  while (buffer_.size() >= kHeaderSize) {
    if (!std::equal(kMagic.begin(), kMagic.end(), buffer_.begin())) {
      fail("invalid HTBR magic");
    }
    if (buffer_[4] != kBridgeProtocolVersion) {
      fail("unsupported HTBR protocol version");
    }
    if (!known_type(buffer_[5])) {
      fail("unknown HTBR message type");
    }
    if (read_u16_be(buffer_.data() + 6U) != 0U) {
      fail("unsupported HTBR flags");
    }
    const auto payload_size = read_u32_be(buffer_.data() + 12U);
    if (payload_size > max_payload_) {
      fail("HTBR payload exceeds configured limit");
    }
    const auto frame_size = kHeaderSize + static_cast<std::size_t>(payload_size);
    if (buffer_.size() < frame_size) {
      break;
    }
    const auto expected_crc = read_u32_be(buffer_.data() + 24U);
    std::vector<std::uint8_t> crc_input(buffer_.begin(),
                                        buffer_.begin() + frame_size);
    std::fill(crc_input.begin() + 24, crc_input.begin() + 28, 0U);
    if (crc32_iso_hdlc(crc_input) != expected_crc) {
      fail("HTBR CRC mismatch");
    }
    Frame frame;
    frame.type = static_cast<MessageType>(buffer_[5]);
    frame.sequence = read_u32_be(buffer_.data() + 8U);
    frame.monotonic_time_ns = read_u64_be(buffer_.data() + 16U);
    frame.payload.assign(buffer_.begin() + kHeaderSize,
                         buffer_.begin() + frame_size);
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + frame_size);
  }
  return frames;
}

std::vector<Frame> ProtocolHandler::feed(
    const std::vector<std::uint8_t>& bytes) {
  return feed(bytes.data(), bytes.size());
}

std::uint32_t ProtocolHandler::dropped_frames() const noexcept {
  return dropped_frames_;
}

void ProtocolHandler::reset() noexcept { buffer_.clear(); }

[[noreturn]] void ProtocolHandler::fail(const std::string& message) {
  ++dropped_frames_;
  buffer_.clear();
  throw ProtocolError(message);
}

std::uint32_t crc32_iso_hdlc(const std::uint8_t* data, std::size_t size) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      const auto mask = static_cast<std::uint32_t>(
          -static_cast<std::int32_t>(crc & 1U));
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return crc ^ 0xFFFFFFFFU;
}

std::uint32_t crc32_iso_hdlc(const std::vector<std::uint8_t>& data) {
  return crc32_iso_hdlc(data.data(), data.size());
}

bool HelloPayload::operator==(const HelloPayload& o) const {
  return min_version == o.min_version && max_version == o.max_version &&
         capabilities == o.capabilities && instance_nonce == o.instance_nonce;
}
bool HelloAckPayload::operator==(const HelloAckPayload& o) const {
  return selected_version == o.selected_version &&
         capabilities == o.capabilities && instance_nonce == o.instance_nonce &&
         sdk_version == o.sdk_version;
}
bool VelocityPayload::operator==(const VelocityPayload& o) const {
  return vx == o.vx && vy == o.vy && vyaw == o.vyaw;
}
bool ModePayload::operator==(const ModePayload& o) const {
  return mode == o.mode;
}
bool EstopPayload::operator==(const EstopPayload& o) const {
  return engage == o.engage;
}
bool AckPayload::operator==(const AckPayload& o) const {
  return request_sequence == o.request_sequence &&
         result_code == o.result_code && text == o.text;
}
bool ErrorPayload::operator==(const ErrorPayload& o) const {
  return request_sequence == o.request_sequence && error == o.error &&
         text == o.text;
}
bool ImuPayload::operator==(const ImuPayload& o) const {
  return device_time == o.device_time && acceleration == o.acceleration &&
         angular_velocity == o.angular_velocity && quaternion == o.quaternion;
}
bool SportPayload::operator==(const SportPayload& o) const {
  return device_time == o.device_time && wheel_speed == o.wheel_speed &&
         sport_status == o.sport_status;
}
bool OdometryPayload::operator==(const OdometryPayload& o) const {
  return device_time == o.device_time && position == o.position &&
         orientation == o.orientation;
}
bool RobotStatePayload::operator==(const RobotStatePayload& o) const {
  return sdk_linked == o.sdk_linked &&
         control_authority == o.control_authority &&
         emergency_stop == o.emergency_stop &&
         camera_available == o.camera_available &&
         odometry_scale_verified == o.odometry_scale_verified &&
         system_status == o.system_status && error_code == o.error_code &&
         warning_code == o.warning_code && sport_status == o.sport_status &&
         battery_percentage == o.battery_percentage &&
         battery_temperature == o.battery_temperature &&
         battery_voltage == o.battery_voltage &&
         battery_cycle_count == o.battery_cycle_count &&
         charge_status == o.charge_status && wheel_speed == o.wheel_speed &&
         last_velocity_sequence == o.last_velocity_sequence &&
         last_error == o.last_error;
}
bool CameraChunkPayload::operator==(const CameraChunkPayload& o) const {
  return stream_id == o.stream_id &&
         receive_time_ns == o.receive_time_ns &&
         datagram_sequence == o.datagram_sequence && data == o.data;
}

std::vector<std::uint8_t> encode_hello(const HelloPayload& p) {
  validate_capabilities(p.capabilities);
  Writer w;
  w.u8(p.min_version);
  w.u8(p.max_version);
  w.u16(0);
  w.u32(p.capabilities);
  w.u32(p.instance_nonce);
  return w.finish();
}
HelloPayload decode_hello(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  HelloPayload p;
  p.min_version = r.u8();
  p.max_version = r.u8();
  if (r.u16() != 0U) {
    throw ProtocolError("HELLO reserved field is non-zero");
  }
  p.capabilities = r.u32();
  validate_capabilities(p.capabilities);
  p.instance_nonce = r.u32();
  r.finish();
  if (p.min_version == 0U || p.min_version > p.max_version) {
    throw ProtocolError("HELLO version range is invalid");
  }
  return p;
}
std::vector<std::uint8_t> encode_hello_ack(const HelloAckPayload& p) {
  validate_capabilities(p.capabilities);
  Writer w;
  w.u8(p.selected_version);
  w.u8(0);
  w.u16(0);
  w.u32(p.capabilities);
  w.u32(p.instance_nonce);
  w.text(p.sdk_version);
  return w.finish();
}
HelloAckPayload decode_hello_ack(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  HelloAckPayload p;
  p.selected_version = r.u8();
  if (r.u8() != 0U || r.u16() != 0U) {
    throw ProtocolError("HELLO_ACK reserved field is non-zero");
  }
  p.capabilities = r.u32();
  validate_capabilities(p.capabilities);
  p.instance_nonce = r.u32();
  p.sdk_version = r.text();
  r.finish();
  return p;
}

void validate_hello_ack(const HelloAckPayload& payload,
                        std::uint32_t expected_nonce,
                        std::uint32_t requested_capabilities) {
  validate_capabilities(payload.capabilities);
  validate_capabilities(requested_capabilities);
  if (payload.selected_version != kBridgeProtocolVersion) {
    throw ProtocolError("agent selected an incompatible HTBR version");
  }
  if (payload.instance_nonce != expected_nonce) {
    throw ProtocolError("agent HELLO_ACK nonce does not match this session");
  }
  if ((payload.capabilities & ~requested_capabilities) != 0U) {
    throw ProtocolError("agent HELLO_ACK contains unrequested capabilities");
  }
}
std::vector<std::uint8_t> encode_velocity(const VelocityPayload& p) {
  Writer w;
  write_f32_array(w, std::array<float, 3>{p.vx, p.vy, p.vyaw});
  return w.finish();
}
VelocityPayload decode_velocity(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  const auto values = read_f32_array<3>(r);
  r.finish();
  return {values[0], values[1], values[2]};
}
std::vector<std::uint8_t> encode_mode(const ModePayload& p) {
  Writer w;
  w.u16(p.mode);
  return w.finish();
}
ModePayload decode_mode(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  ModePayload p{r.u16()};
  r.finish();
  return p;
}
std::vector<std::uint8_t> encode_estop(const EstopPayload& p) {
  Writer w;
  w.u8(p.engage ? 1U : 0U);
  return w.finish();
}
EstopPayload decode_estop(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  const auto value = r.u8();
  r.finish();
  if (value > 1U) {
    throw ProtocolError("CMD_ESTOP value is not boolean");
  }
  return {value == 1U};
}
std::vector<std::uint8_t> encode_ack(const AckPayload& p) {
  Writer w;
  w.u32(p.request_sequence);
  w.u16(p.result_code);
  w.text(p.text);
  return w.finish();
}
AckPayload decode_ack(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  AckPayload p{r.u32(), r.u16(), r.text()};
  r.finish();
  return p;
}
std::vector<std::uint8_t> encode_error(const ErrorPayload& p) {
  if (static_cast<std::uint16_t>(p.error) == 0U) {
    throw ProtocolError("ERROR code cannot be zero");
  }
  Writer w;
  w.u32(p.request_sequence);
  w.u16(static_cast<std::uint16_t>(p.error));
  w.text(p.text);
  return w.finish();
}
ErrorPayload decode_error(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  ErrorPayload p;
  p.request_sequence = r.u32();
  const auto code = r.u16();
  if (code == 0U || code > static_cast<std::uint16_t>(BridgeError::SdkCallFailed)) {
    throw ProtocolError("ERROR code is invalid");
  }
  p.error = static_cast<BridgeError>(code);
  p.text = r.text();
  r.finish();
  return p;
}
std::vector<std::uint8_t> encode_imu(const ImuPayload& p) {
  Writer w;
  w.u64(p.device_time);
  write_f32_array(w, p.acceleration);
  write_f32_array(w, p.angular_velocity);
  write_f32_array(w, p.quaternion);
  return w.finish();
}
ImuPayload decode_imu(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  ImuPayload p;
  p.device_time = r.u64();
  p.acceleration = read_f32_array<3>(r);
  p.angular_velocity = read_f32_array<3>(r);
  p.quaternion = read_f32_array<4>(r);
  r.finish();
  return p;
}
std::vector<std::uint8_t> encode_sport(const SportPayload& p) {
  Writer w;
  w.u64(p.device_time);
  write_f32_array(w, p.wheel_speed);
  w.u16(p.sport_status);
  return w.finish();
}
SportPayload decode_sport(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  SportPayload p;
  p.device_time = r.u64();
  p.wheel_speed = read_f32_array<4>(r);
  p.sport_status = r.u16();
  r.finish();
  return p;
}
std::vector<std::uint8_t> encode_odometry(const OdometryPayload& p) {
  Writer w;
  w.u64(p.device_time);
  write_f64_array(w, p.position);
  write_f64_array(w, p.orientation);
  return w.finish();
}
OdometryPayload decode_odometry(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  OdometryPayload p;
  p.device_time = r.u64();
  p.position = read_f64_array<3>(r);
  p.orientation = read_f64_array<4>(r);
  r.finish();
  return p;
}
std::vector<std::uint8_t> encode_robot_state(const RobotStatePayload& p) {
  Writer w;
  w.u8(p.sdk_linked ? 1U : 0U);
  w.u8(p.control_authority ? 1U : 0U);
  w.u8(p.emergency_stop ? 1U : 0U);
  w.u8(p.camera_available ? 1U : 0U);
  w.u8(p.odometry_scale_verified ? 1U : 0U);
  w.u8(p.system_status);
  w.u32(p.error_code);
  w.u32(p.warning_code);
  w.u16(p.sport_status);
  write_f32_array(w, std::array<float, 3>{p.battery_percentage,
                                          p.battery_temperature,
                                          p.battery_voltage});
  w.u16(p.battery_cycle_count);
  w.u16(p.charge_status);
  write_f32_array(w, p.wheel_speed);
  w.u32(p.last_velocity_sequence);
  w.text(p.last_error);
  return w.finish();
}
RobotStatePayload decode_robot_state(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  RobotStatePayload p;
  p.sdk_linked = r.u8() != 0U;
  p.control_authority = r.u8() != 0U;
  p.emergency_stop = r.u8() != 0U;
  p.camera_available = r.u8() != 0U;
  p.odometry_scale_verified = r.u8() != 0U;
  p.system_status = r.u8();
  p.error_code = r.u32();
  p.warning_code = r.u32();
  p.sport_status = r.u16();
  const auto battery = read_f32_array<3>(r);
  p.battery_percentage = battery[0];
  p.battery_temperature = battery[1];
  p.battery_voltage = battery[2];
  p.battery_cycle_count = r.u16();
  p.charge_status = r.u16();
  p.wheel_speed = read_f32_array<4>(r);
  p.last_velocity_sequence = r.u32();
  p.last_error = r.text();
  r.finish();
  return p;
}
std::vector<std::uint8_t> encode_camera_chunk(const CameraChunkPayload& p) {
  Writer w;
  w.u32(p.stream_id);
  w.u64(p.receive_time_ns);
  w.u32(p.datagram_sequence);
  w.raw(p.data);
  return w.finish();
}
CameraChunkPayload decode_camera_chunk(const std::vector<std::uint8_t>& bytes) {
  Reader r(bytes);
  CameraChunkPayload p;
  p.stream_id = r.u32();
  p.receive_time_ns = r.u64();
  p.datagram_sequence = r.u32();
  p.data = r.rest();
  r.finish();
  return p;
}

std::optional<OdometryPacket> parse_odometry_packet(
    const std::vector<std::uint8_t>& datagram, PackingMode packing,
    double position_scale, double quaternion_scale) {
  if (!std::isfinite(position_scale) || position_scale <= 0.0 ||
      !std::isfinite(quaternion_scale) || quaternion_scale <= 0.0) {
    return std::nullopt;
  }

  const auto parse_layout = [&](bool aligned) -> std::optional<OdometryPacket> {
    // TODO:参考手册第26页补充实现：确认UDP 6101里程计的packing和字节序。
    const std::size_t time_offset = aligned ? 8U : 2U;
    const std::size_t position_offset = aligned ? 16U : 10U;
    const std::size_t quaternion_offset = aligned ? 40U : 34U;
    const std::size_t tail_offset = aligned ? 72U : 66U;
    const std::size_t expected_size = aligned ? 80U : 68U;
    if (datagram.size() != expected_size ||
        read_u16_le(datagram.data()) != 0xAA55U ||
        read_u16_le(datagram.data() + tail_offset) != 0xFF00U) {
      return std::nullopt;
    }

    OdometryPacket packet;
    packet.device_time = read_u64_le(datagram.data() + time_offset);
    // TODO:参考手册第26页补充实现：用实机数据标定位置与四元数比例。
    for (std::size_t i = 0; i < packet.position.size(); ++i) {
      packet.position[i] = static_cast<double>(read_i64_le(
                               datagram.data() + position_offset + 8U * i)) *
                           position_scale;
      if (!std::isfinite(packet.position[i])) {
        return std::nullopt;
      }
    }
    double norm_squared = 0.0;
    for (std::size_t i = 0; i < packet.orientation.size(); ++i) {
      packet.orientation[i] =
          static_cast<double>(read_i64_le(
              datagram.data() + quaternion_offset + 8U * i)) *
          quaternion_scale;
      norm_squared += packet.orientation[i] * packet.orientation[i];
    }
    if (!std::isfinite(norm_squared) || norm_squared <= 1e-24) {
      return std::nullopt;
    }
    const auto norm = std::sqrt(norm_squared);
    for (auto& component : packet.orientation) {
      component /= norm;
    }
    return packet;
  };

  if (packing == PackingMode::Packed) {
    return parse_layout(false);
  }
  if (packing == PackingMode::NaturalAligned) {
    return parse_layout(true);
  }
  if (auto packed = parse_layout(false)) {
    return packed;
  }
  return parse_layout(true);
}

std::optional<PointCloudPacket> parse_point_cloud_packet(
    const std::vector<std::uint8_t>& datagram, PackingMode packing,
    double coordinate_scale) {
  if (!std::isfinite(coordinate_scale) || coordinate_scale <= 0.0) {
    return std::nullopt;
  }

  const auto parse_layout = [&](bool aligned)
      -> std::optional<PointCloudPacket> {
    // TODO:参考手册第24页补充实现：确认UDP 6100点云的packing和字节序。
    const std::size_t time_offset = aligned ? 8U : 2U;
    const std::size_t frame_offset = aligned ? 16U : 10U;
    const std::size_t data_offset = aligned ? 20U : 14U;
    const std::size_t count_offset = aligned ? 24U : 18U;
    const std::size_t points_offset = aligned ? 28U : 20U;
    if (datagram.size() < points_offset + 2U ||
        read_u16_le(datagram.data()) != 0xAA55U) {
      return std::nullopt;
    }
    const auto count = read_u16_le(datagram.data() + count_offset);
    if (count > 50U) {
      return std::nullopt;
    }
    constexpr std::size_t kPointSize = 28U;
    const auto dynamic_tail = points_offset + kPointSize * count;
    const auto fixed_tail = points_offset + kPointSize * 50U;
    std::size_t tail_offset{};
    const auto tail_matches = [&](std::size_t offset) {
      return offset + 2U <= datagram.size() &&
             read_u16_le(datagram.data() + offset) == 0xFF00U;
    };
    if (tail_matches(dynamic_tail) &&
        (datagram.size() == dynamic_tail + 2U ||
         (aligned && datagram.size() <= dynamic_tail + 8U))) {
      tail_offset = dynamic_tail;
    } else if (tail_matches(fixed_tail) &&
               (datagram.size() == fixed_tail + 2U ||
                (aligned && datagram.size() <= fixed_tail + 8U))) {
      tail_offset = fixed_tail;
    } else {
      return std::nullopt;
    }
    (void)tail_offset;

    PointCloudPacket packet;
    packet.device_time = read_u64_le(datagram.data() + time_offset);
    packet.total_points = read_u32_le(datagram.data() + frame_offset);
    packet.packet_index = read_u32_le(datagram.data() + data_offset);
    packet.points.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
      const auto* point = datagram.data() + points_offset + i * kPointSize;
      ProtocolLidarPoint decoded;
      decoded.x = static_cast<double>(read_i32_le(point)) * coordinate_scale;
      decoded.y =
          static_cast<double>(read_i32_le(point + 4U)) * coordinate_scale;
      decoded.z =
          static_cast<double>(read_i32_le(point + 8U)) * coordinate_scale;
      for (std::size_t channel = 0; channel < decoded.rgba.size(); ++channel) {
        decoded.rgba[channel] = read_u32_le(point + 12U + channel * 4U);
      }
      if (!std::isfinite(decoded.x) || !std::isfinite(decoded.y) ||
          !std::isfinite(decoded.z)) {
        return std::nullopt;
      }
      packet.points.push_back(decoded);
    }
    return packet;
  };

  if (packing == PackingMode::Packed) {
    return parse_layout(false);
  }
  if (packing == PackingMode::NaturalAligned) {
    return parse_layout(true);
  }
  if (auto packed = parse_layout(false)) {
    return packed;
  }
  return parse_layout(true);
}

}  // namespace hypertron_ros2_bridge
