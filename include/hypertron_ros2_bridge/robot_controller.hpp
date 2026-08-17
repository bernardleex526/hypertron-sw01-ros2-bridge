#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>

namespace hypertron_ros2_bridge {

// Controller-owned protocol value types. These used to live in the retired
// HTBR protocol handler; the direct driver only needs the safety-relevant
// subset, so they are defined here to keep the controller dependency-free.
enum class BridgeError : std::uint16_t {
  Protocol = 1,
  FeatureUnavailable = 2,
  InvalidCommand = 3,
  Timeout = 4,
  SdkDisconnected = 5,
  NoControlAuthority = 6,
  EmergencyStopLatched = 7,
  RobotSystemError = 8,
  SdkCallFailed = 9,
};

struct VelocityPayload {
  float vx{};
  float vy{};
  float vyaw{};
  bool operator==(const VelocityPayload& o) const {
    return vx == o.vx && vy == o.vy && vyaw == o.vyaw;
  }
};

class IMonotonicClock {
 public:
  using time_point = std::chrono::steady_clock::time_point;
  virtual ~IMonotonicClock() = default;
  virtual time_point now() const = 0;
};

class SteadyMonotonicClock final : public IMonotonicClock {
 public:
  time_point now() const override { return std::chrono::steady_clock::now(); }
};

struct ControllerConfig {
  std::chrono::milliseconds command_deadman{100};
};

struct ControllerStatus {
  bool sdk_linked{};
  bool control_authority{};
  std::uint16_t sport_status{};
  std::uint32_t error_code{};
};

struct VelocityDecision {
  bool accepted{};
  VelocityPayload value{};
  BridgeError error{BridgeError::Protocol};
  std::string reason;
};

struct ModeDecision {
  bool accepted{};
  std::uint16_t mode{};
  BridgeError error{BridgeError::Protocol};
  std::string reason;
};

struct EstopDecision {
  bool accepted{};
  bool latched{};
  std::string reason;
};

// Implements the ASTRALL safety gates documented on manual pages 16-22:
// Move is legal only with an explicit driver-ready handshake, an active link,
// SDK authority, Move sport state, no system error and no software
// emergency-stop latch.
class RobotController {
 public:
  RobotController(ControllerConfig config, IMonotonicClock& clock);

  VelocityDecision accept_velocity(VelocityPayload command);
  VelocityDecision velocity_for_tick() const;
  EstopDecision trigger_estop();
  EstopDecision clear_estop();
  ModeDecision request_mode(std::string_view name);
  void complete_mode_transition(bool success);
  // Arms the driver readiness gate. The direct runtime calls this only after
  // a fresh connection has been fully re-established; a ready status reported
  // by the robot alone never re-arms motion.
  void set_driver_ready(bool ready);
  // True while a mode transition is pending and motion is gated.
  bool mode_transition_pending() const;
  // Clears readiness, pending mode and velocity after a connection loss.
  // The software emergency-stop latch is deliberately preserved.
  void invalidate_connection();
  void update_robot_state(const ControllerStatus& state);

  bool joint_interface_available() const noexcept { return false; }
  bool emergency_stop_latched() const;
  ControllerStatus status() const;

 private:
  VelocityDecision rejection_locked() const;
  bool movement_ready_locked() const;
  void force_zero_locked();

  ControllerConfig config_;
  IMonotonicClock& clock_;
  mutable std::mutex mutex_;
  ControllerStatus status_;
  VelocityPayload latest_velocity_{};
  IMonotonicClock::time_point last_velocity_time_{};
  bool has_velocity_{false};
  bool estop_latched_{false};
  bool driver_ready_{false};
  bool motion_transition_gate_{false};
  bool mode_transition_pending_{false};
  std::uint16_t pending_mode_{0};
};

}  // namespace hypertron_ros2_bridge
