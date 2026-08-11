#include "hypertron_ros2_bridge/data_receiver.hpp"

#include <cmath>
#include <limits>

namespace hypertron_ros2_bridge {
namespace {

std::uint64_t resolve_timestamp(std::uint64_t device,
                                std::uint64_t receive,
                                TimestampSource source) {
  constexpr auto kThousand = std::uint64_t{1000};
  constexpr auto kMillion = std::uint64_t{1000000};
  if (source == TimestampSource::Receive) return receive;
  if (source == TimestampSource::DeviceNanoseconds) return device;
  const auto multiplier = source == TimestampSource::DeviceMicroseconds
                              ? kThousand
                              : kMillion;
  if (device > std::numeric_limits<std::uint64_t>::max() / multiplier) {
    throw MappingError("device timestamp overflows nanoseconds");
  }
  return device * multiplier;
}

template <std::size_t N>
void require_finite(const std::array<float, N>& values,
                    const char* field) {
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      throw MappingError(std::string(field) + " contains a non-finite value");
    }
  }
}

template <std::size_t N>
void require_finite(const std::array<double, N>& values,
                    const char* field) {
  for (const auto value : values) {
    if (!std::isfinite(value)) {
      throw MappingError(std::string(field) + " contains a non-finite value");
    }
  }
}

std::array<double, 4> normalize(std::array<double, 4> quaternion) {
  require_finite(quaternion, "quaternion");
  double squared{};
  for (const auto component : quaternion) squared += component * component;
  if (!std::isfinite(squared) || squared <= 1e-24) {
    throw MappingError("quaternion norm is zero");
  }
  const auto norm = std::sqrt(squared);
  for (auto& component : quaternion) component /= norm;
  return quaternion;
}

std::array<double, 9> diagonal(const std::array<double, 3>& values) {
  require_finite(values, "covariance");
  std::array<double, 9> covariance{};
  covariance[0] = values[0];
  covariance[4] = values[1];
  covariance[8] = values[2];
  return covariance;
}

}  // namespace

ImuSample to_imu_sample(const ImuPayload& payload,
                        std::uint64_t receive_time_ns,
                        const DataReceiverConfig& config) {
  require_finite(payload.acceleration, "linear acceleration");
  require_finite(payload.angular_velocity, "angular velocity");
  require_finite(payload.quaternion, "quaternion");
  ImuSample sample;
  sample.timestamp_ns = resolve_timestamp(payload.device_time, receive_time_ns,
                                          config.timestamp_source);
  sample.frame_id = config.imu_frame;
  for (std::size_t i = 0; i < 3; ++i) {
    sample.linear_acceleration[i] = payload.acceleration[i];
    sample.angular_velocity[i] = payload.angular_velocity[i];
  }
  if (config.quaternion_order == QuaternionOrder::Xyzw) {
    for (std::size_t i = 0; i < 4; ++i) {
      sample.orientation[i] = payload.quaternion[i];
    }
  } else {
    sample.orientation = {payload.quaternion[1], payload.quaternion[2],
                          payload.quaternion[3], payload.quaternion[0]};
  }
  sample.orientation = normalize(sample.orientation);
  sample.orientation_covariance =
      diagonal(config.orientation_covariance_diagonal);
  sample.angular_velocity_covariance =
      diagonal(config.angular_velocity_covariance_diagonal);
  sample.linear_acceleration_covariance =
      diagonal(config.linear_acceleration_covariance_diagonal);
  return sample;
}

OdometrySample to_odometry_sample(const OdometryPayload& payload,
                                  std::uint64_t receive_time_ns,
                                  const DataReceiverConfig& config) {
  require_finite(payload.position, "odometry position");
  require_finite(payload.orientation, "odometry quaternion");
  OdometrySample sample;
  sample.timestamp_ns = resolve_timestamp(payload.device_time, receive_time_ns,
                                          config.odometry_timestamp_source);
  sample.frame_id = config.odom_frame;
  sample.child_frame_id = config.base_frame;
  sample.position = payload.position;
  sample.orientation = normalize(payload.orientation);
  return sample;
}

bool CameraIngestState::accept(const CameraChunkPayload& payload) {
  if (!enabled_) {
    disabled_drops_.fetch_add(1);
    return false;
  }
  return !payload.data.empty();
}

bool effective_emergency_stop(bool pc_latched, bool agent_latched) noexcept {
  return pc_latched || agent_latched;
}

}  // namespace hypertron_ros2_bridge
