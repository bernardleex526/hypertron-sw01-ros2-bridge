#include "hypertron_ros2_bridge/astrall_sdk_adapter.hpp"

#include <algorithm>
#include <mutex>
#include <utility>

#ifdef HYPERTRON_WITH_ASTRALL_SDK
#include "interface.h"
#endif

namespace hypertron_ros2_bridge {

struct AstrallSdkAdapter::Impl {
  mutable std::mutex mutex;
  SdkCallbacks callbacks;
  SdkSnapshot latest;
  std::string version{"ASTRALL unavailable"};
  bool initialized{false};
};

AstrallSdkAdapter::AstrallSdkAdapter() : impl_(std::make_unique<Impl>()) {}
AstrallSdkAdapter::~AstrallSdkAdapter() { deinit(); }

std::uint16_t AstrallSdkAdapter::init(const SdkCallbacks& callbacks,
                                      std::uint32_t timeout_ms) {
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->callbacks = callbacks;
  }
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  AstrallConfig config;
  config.heartbeatCb = [](void*, std::uint16_t) {};
  config.sdkStatusCb = [this](void* data, std::uint16_t length) {
    if (data == nullptr || length != sizeof(AstrallSdkStatus)) {
      return;
    }
    const auto* vendor = static_cast<const AstrallSdkStatus*>(data);
    std::function<void(bool, bool)> callback;
    bool linked{};
    bool authority{};
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->latest.sdk_linked = vendor->link != 0U;
      impl_->latest.control_authority = vendor->ctrlAuthority != 0U;
      linked = impl_->latest.sdk_linked;
      authority = impl_->latest.control_authority;
      callback = impl_->callbacks.on_link;
    }
    if (callback) {
      callback(linked, authority);
    }
  };

  const auto result = AstrallSdkInit(config, timeout_ms);
  if (result != ASTRALL_RES_SUCCESSED) {
    return result;
  }
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->initialized = true;
  }

  AstrallDeviceInfo device;
  if (AstrallGetDeviceInfo(device, timeout_ms) == ASTRALL_RES_SUCCESSED) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->version = device.version;
  }

  auto imu_callback = [this](void* data, std::uint16_t length) {
    if (data == nullptr || length != sizeof(AstrallImuData)) {
      return;
    }
    const auto* vendor = static_cast<const AstrallImuData*>(data);
    ImuPayload payload;
    payload.device_time =
        vendor->timestamp < 0 ? 0U : static_cast<std::uint64_t>(vendor->timestamp);
    std::copy_n(vendor->accelerometer, 3, payload.acceleration.begin());
    std::copy_n(vendor->gyroscope, 3, payload.angular_velocity.begin());
    std::copy_n(vendor->quaternion, 4, payload.quaternion.begin());
    std::function<void(ImuPayload)> callback;
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      callback = impl_->callbacks.on_imu;
    }
    if (callback) {
      callback(std::move(payload));
    }
  };
  const auto imu_result = AstrallSubscriptionData(
      ASTRALL_SUB_TOPIC_ID_IMU, ASTRALL_SUB_FREQ_125HZ, imu_callback,
      timeout_ms);
  if (imu_result != ASTRALL_RES_SUCCESSED) {
    deinit();
    return imu_result;
  }

  auto sport_callback = [this](void* data, std::uint16_t length) {
    if (data == nullptr || length != sizeof(AstrallSportData)) {
      return;
    }
    const auto* vendor = static_cast<const AstrallSportData*>(data);
    SportPayload payload;
    payload.device_time =
        vendor->timestamp < 0 ? 0U : static_cast<std::uint64_t>(vendor->timestamp);
    std::copy_n(vendor->wheelSpeed, 4, payload.wheel_speed.begin());
    std::function<void(SportPayload)> callback;
    {
      std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->latest.wheel_speed = payload.wheel_speed;
      payload.sport_status = impl_->latest.sport_status;
      callback = impl_->callbacks.on_sport;
    }
    if (callback) {
      callback(std::move(payload));
    }
  };
  const auto sport_result = AstrallSubscriptionData(
      ASTRALL_SUB_TOPIC_ID_SPORT, ASTRALL_SUB_FREQ_50HZ, sport_callback,
      timeout_ms);
  if (sport_result != ASTRALL_RES_SUCCESSED) {
    deinit();
    return sport_result;
  }
  return result;
#else
  (void)timeout_ms;
  return 0x8011U;
#endif
}

void AstrallSdkAdapter::deinit() noexcept {
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  bool should_deinit{};
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    should_deinit = impl_->initialized;
    impl_->initialized = false;
    impl_->latest.sdk_linked = false;
    impl_->latest.control_authority = false;
  }
  if (should_deinit) {
    AstrallSdkDeinit();
  }
#endif
}

std::uint16_t AstrallSdkAdapter::heartbeat(std::uint32_t timeout_ms) {
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  return AstrallHeartbeat(timeout_ms);
#else
  (void)timeout_ms;
  return 0x8011U;
#endif
}

std::uint16_t AstrallSdkAdapter::acquire_sdk_control(
    std::uint32_t timeout_ms) {
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  return AstrallAuthControl(ASTRALL_AUTH_SDK, timeout_ms);
#else
  (void)timeout_ms;
  return 0x8011U;
#endif
}

std::uint16_t AstrallSdkAdapter::configure_udp_streams(
    bool camera, bool lidar, std::uint32_t timeout_ms) {
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  const auto discard_callback = [](void*, std::uint16_t) {};
  if (camera) {
    const auto result = AstrallSubscriptionData(
        ASTRALL_SUB_TOPIC_ID_CAMERA_RGB, ASTRALL_SUB_FREQ_1HZ,
        discard_callback, timeout_ms);
    if (result != ASTRALL_RES_SUCCESSED) return result;
  }
  if (lidar) {
    const auto result = AstrallSubscriptionData(
        ASTRALL_SUB_TOPIC_ID_LIDAR, ASTRALL_SUB_FREQ_1HZ,
        discard_callback, timeout_ms);
    if (result != ASTRALL_RES_SUCCESSED) return result;
  }
  return ASTRALL_RES_SUCCESSED;
#else
  (void)camera;
  (void)lidar;
  (void)timeout_ms;
  return 0x8011U;
#endif
}

std::uint16_t AstrallSdkAdapter::move(float vx, float vy, float vyaw,
                                      std::uint32_t timeout_ms) {
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  return AstrallMove(vx, vy, vyaw, timeout_ms);
#else
  (void)vx;
  (void)vy;
  (void)vyaw;
  (void)timeout_ms;
  return 0x8011U;
#endif
}

std::uint16_t AstrallSdkAdapter::set_mode(std::uint16_t mode,
                                          std::uint32_t timeout_ms) {
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  return AstrallSportModeControl(static_cast<AstrallSportModeCmd>(mode),
                                 timeout_ms);
#else
  (void)mode;
  (void)timeout_ms;
  return 0x8011U;
#endif
}

SdkSnapshot AstrallSdkAdapter::snapshot() {
#ifdef HYPERTRON_WITH_ASTRALL_SDK
  AstrallSystemStatus system;
  AstrallPowerStatus power;
  const bool system_ok =
      AstrallGetSystemStatus(system) == ASTRALL_RES_SUCCESSED;
  const bool power_ok = AstrallGetPowerStatus(power) == ASTRALL_RES_SUCCESSED;
  const auto sport = AstrallGetSportStatus();
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (system_ok) {
    impl_->latest.system_status = static_cast<std::uint8_t>(system.sysStatus);
    impl_->latest.error_code = static_cast<std::uint32_t>(system.errorCode);
    impl_->latest.warning_code = static_cast<std::uint32_t>(system.warnCode);
  }
  if (power_ok) {
    impl_->latest.battery_percentage = power.soc;
    impl_->latest.battery_temperature = power.temp;
    impl_->latest.battery_voltage = power.voltage;
    impl_->latest.battery_cycle_count = power.cycleCount;
    impl_->latest.charge_status = power.charged;
  }
  impl_->latest.sport_status = sport;
  return impl_->latest;
#else
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->latest;
#endif
}

std::string AstrallSdkAdapter::sdk_version() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->version;
}

}  // namespace hypertron_ros2_bridge
