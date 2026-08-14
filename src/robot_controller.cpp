#include "hypertron_ros2_bridge/robot_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace hypertron_ros2_bridge {
namespace {

constexpr std::uint16_t kSportStateMove = 0xB104U;

std::string normalized_mode(std::string_view input) {
  std::size_t first = 0;
  while (first < input.size() &&
         std::isspace(static_cast<unsigned char>(input[first])) != 0) {
    ++first;
  }
  std::size_t last = input.size();
  while (last > first &&
         std::isspace(static_cast<unsigned char>(input[last - 1U])) != 0) {
    --last;
  }
  std::string value(input.substr(first, last - first));
  std::transform(value.begin(), value.end(), value.begin(), [](char c) {
    return static_cast<char>(
        std::tolower(static_cast<unsigned char>(c)));
  });
  return value;
}

}  // namespace

RobotController::RobotController(ControllerConfig config,
                                 IMonotonicClock& clock)
    : config_(config), clock_(clock) {
  if (config_.command_deadman.count() <= 0) {
    throw std::invalid_argument("command deadman must be positive");
  }
}

VelocityDecision RobotController::accept_velocity(VelocityPayload command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) ||
      !std::isfinite(command.vyaw)) {
    force_zero_locked();
    return {false, {}, BridgeError::InvalidCommand,
            "velocity contains a non-finite value"};
  }
  if (!movement_ready_locked()) {
    force_zero_locked();
    return rejection_locked();
  }
  command.vx = std::clamp(command.vx, -1.0F, 1.0F);
  command.vy = std::clamp(command.vy, -1.0F, 1.0F);
  command.vyaw = std::clamp(command.vyaw, -1.0F, 1.0F);
  latest_velocity_ = command;
  last_velocity_time_ = clock_.now();
  has_velocity_ = true;
  return {true, command, BridgeError::Protocol, {}};
}

VelocityDecision RobotController::velocity_for_tick() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!movement_ready_locked() || !has_velocity_ ||
      clock_.now() - last_velocity_time_ > config_.command_deadman) {
    return {false, {}, BridgeError::Timeout, "velocity deadman or safety gate"};
  }
  return {true, latest_velocity_, BridgeError::Protocol, {}};
}

EstopDecision RobotController::trigger_estop() {
  std::lock_guard<std::mutex> lock(mutex_);
  estop_latched_ = true;
  force_zero_locked();
  return {true, true, "software emergency stop latched"};
}

EstopDecision RobotController::clear_estop() {
  std::lock_guard<std::mutex> lock(mutex_);
  estop_latched_ = false;
  force_zero_locked();
  return {true, false,
          "software emergency stop cleared; a new velocity is required"};
}

ModeDecision RobotController::request_mode(std::string_view name) {
  static const std::unordered_map<std::string, std::uint16_t> kModes{
      {"damping", 0xA101U},     {"stand", 0xA102U},
      {"down", 0xA103U},        {"move", 0xA104U},
      {"auto_charge", 0xA105U}, {"exit_charge", 0xA106U},
      {"recover", 0xA1FFU},     {"recovery", 0xA1FFU},
  };
  const auto normalized = normalized_mode(name);
  const auto mode = kModes.find(normalized);
  if (mode == kModes.end()) {
    return {false, 0, BridgeError::InvalidCommand,
            "unknown robot mode: " + normalized};
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (!driver_ready_) {
    return {false, mode->second, BridgeError::Protocol,
            "driver readiness gate is not armed"};
  }
  if (mode_transition_pending_) {
    return {false, mode->second, BridgeError::InvalidCommand,
            "another mode transition is pending"};
  }
  if (!status_.sdk_linked) {
    return {false, mode->second, BridgeError::SdkDisconnected,
            "ASTRALL SDK is disconnected"};
  }
  if (status_.error_code != 0U) {
    return {false, mode->second, BridgeError::RobotSystemError,
            "robot system error is active"};
  }
  if (!status_.control_authority) {
    return {false, mode->second, BridgeError::NoControlAuthority,
            "ASTRALL control authority is unavailable"};
  }
  if (estop_latched_ && mode->second != 0xA101U) {
    return {false, mode->second, BridgeError::EmergencyStopLatched,
            "software emergency stop is latched"};
  }
  mode_transition_pending_ = true;
  pending_mode_ = mode->second;
  motion_transition_gate_ = true;
  force_zero_locked();
  return {true, mode->second, BridgeError::Protocol, {}};
}

void RobotController::complete_mode_transition(bool success) {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool entered_move = success && mode_transition_pending_ &&
                            pending_mode_ == 0xA104U;
  mode_transition_pending_ = false;
  pending_mode_ = 0;
  motion_transition_gate_ = !entered_move;
  force_zero_locked();
}

void RobotController::set_driver_ready(bool ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  driver_ready_ = ready;
  if (!ready) {
    mode_transition_pending_ = false;
    pending_mode_ = 0;
    motion_transition_gate_ = true;
    force_zero_locked();
  }
}

bool RobotController::mode_transition_pending() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_transition_pending_;
}

void RobotController::invalidate_connection() {
  std::lock_guard<std::mutex> lock(mutex_);
  driver_ready_ = false;
  mode_transition_pending_ = false;
  pending_mode_ = 0;
  motion_transition_gate_ = true;
  force_zero_locked();
  // estop_latched_ is deliberately preserved: a connection loss must not
  // silently clear a software emergency stop.
}

void RobotController::update_robot_state(const ControllerStatus& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool was_ready = movement_ready_locked();
  status_ = state;
  if (!status_.sdk_linked || !status_.control_authority ||
      status_.error_code != 0U) {
    mode_transition_pending_ = false;
    pending_mode_ = 0;
    motion_transition_gate_ = true;
  }
  if (was_ready != movement_ready_locked() || !movement_ready_locked()) {
    force_zero_locked();
  }
}

bool RobotController::reject_joint_command(std::string_view reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++rejected_joint_commands_;
  last_joint_rejection_ = std::string(reason);
  return false;
}

std::uint32_t RobotController::rejected_joint_commands() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return rejected_joint_commands_;
}

bool RobotController::emergency_stop_latched() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return estop_latched_;
}

ControllerStatus RobotController::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

VelocityDecision RobotController::rejection_locked() const {
  if (!driver_ready_) {
    return {false, {}, BridgeError::Protocol,
            "driver readiness gate is not armed"};
  }
  if (!status_.sdk_linked) {
    return {false, {}, BridgeError::SdkDisconnected,
            "ASTRALL SDK is disconnected"};
  }
  if (status_.error_code != 0U) {
    return {false, {}, BridgeError::RobotSystemError,
            "robot system error is active"};
  }
  if (!status_.control_authority) {
    return {false, {}, BridgeError::NoControlAuthority,
            "ASTRALL control authority is unavailable"};
  }
  if (estop_latched_) {
    return {false, {}, BridgeError::EmergencyStopLatched,
            "software emergency stop is latched"};
  }
  if (motion_transition_gate_ || mode_transition_pending_) {
    return {false, {}, BridgeError::InvalidCommand,
            "motion is gated by a mode transition"};
  }
  return {false, {}, BridgeError::InvalidCommand,
          "robot is not in all-terrain Move state"};
}

bool RobotController::movement_ready_locked() const {
  return driver_ready_ && !motion_transition_gate_ &&
         !mode_transition_pending_ && status_.sdk_linked &&
         status_.control_authority &&
         status_.sport_status == kSportStateMove && status_.error_code == 0U &&
         !estop_latched_;
}

void RobotController::force_zero_locked() {
  latest_velocity_ = {};
  has_velocity_ = false;
  last_velocity_time_ = clock_.now();
}

}  // namespace hypertron_ros2_bridge
