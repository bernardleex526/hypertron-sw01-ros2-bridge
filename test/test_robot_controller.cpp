#include <chrono>
#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/robot_controller.hpp"

namespace hypertron_ros2_bridge {
namespace {
using namespace std::chrono_literals;

class ManualClock final : public IMonotonicClock {
 public:
  time_point now() const override { return now_; }
  void advance(std::chrono::milliseconds duration) { now_ += duration; }

 private:
  time_point now_{};
};

ControllerStatus ready_state() {
  ControllerStatus state;
  state.sdk_linked = true;
  state.control_authority = true;
  state.sport_status = 0xB104U;
  return state;
}

TEST(RobotController, ClampsFiniteVelocityAndDeadmanReturnsZero) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.set_driver_ready(true);
  controller.update_robot_state(ready_state());
  EXPECT_EQ(controller.accept_velocity({2.0F, -2.0F, 0.5F}).value,
            (VelocityPayload{1.0F, -1.0F, 0.5F}));
  clock.advance(101ms);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
}

TEST(RobotController, NonFiniteInputForcesZero) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.set_driver_ready(true);
  controller.update_robot_state(ready_state());
  const auto result =
      controller.accept_velocity({NAN, 0.0F, 0.0F});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.error, BridgeError::InvalidCommand);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
}

TEST(RobotController, EstopLatchesAndClearDoesNotRestoreVelocity) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.set_driver_ready(true);
  controller.update_robot_state(ready_state());
  controller.accept_velocity({0.5F, 0.0F, 0.0F});
  EXPECT_TRUE(controller.trigger_estop().accepted);
  EXPECT_FALSE(controller.accept_velocity({0.2F, 0.0F, 0.0F}).accepted);
  EXPECT_TRUE(controller.clear_estop().accepted);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
}

TEST(RobotController, MapsAllDocumentedModesCaseInsensitively) {
  struct ModeCase {
    const char* name;
    std::uint16_t expected;
  };
  const ModeCase kCases[] = {
      {" DAMPING ", 0xA101U}, {"Stand", 0xA102U},   {"down", 0xA103U},
      {"MOVE", 0xA104U},      {"auto_charge", 0xA105U},
      {"exit_charge", 0xA106U}, {"recovery", 0xA1FFU}, {"recover", 0xA1FFU},
  };
  // Each mode uses a fresh controller: a pending transition from the previous
  // request must not mask acceptance of this one.
  for (const ModeCase& mode_case : kCases) {
    ManualClock clock;
    RobotController controller({100ms}, clock);
    controller.set_driver_ready(true);
    auto state = ready_state();
    state.sport_status = 0xB102U;
    controller.update_robot_state(state);
    const auto result = controller.request_mode(mode_case.name);
    EXPECT_TRUE(result.accepted) << mode_case.name;
    EXPECT_EQ(result.mode, mode_case.expected) << mode_case.name;
  }

  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.set_driver_ready(true);
  auto state = ready_state();
  state.sport_status = 0xB102U;
  controller.update_robot_state(state);
  const auto result = controller.request_mode("run");
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.error, BridgeError::InvalidCommand);
}

TEST(RobotController, AuthorityLossAndFaultForceZero) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.set_driver_ready(true);
  auto state = ready_state();
  controller.update_robot_state(state);
  ASSERT_TRUE(controller.accept_velocity({0.5F, 0, 0}).accepted);
  state.control_authority = false;
  controller.update_robot_state(state);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
  EXPECT_FALSE(controller.accept_velocity({0.5F, 0, 0}).accepted);

  state.control_authority = true;
  state.error_code = 1;
  controller.update_robot_state(state);
  EXPECT_FALSE(controller.accept_velocity({0.5F, 0, 0}).accepted);
  EXPECT_EQ(controller.request_mode("move").error,
            BridgeError::RobotSystemError);
}

TEST(RobotController, JointPlaceholderRejectsWithoutCommand) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  EXPECT_FALSE(controller.reject_joint_command("vendor SDK unavailable"));
  EXPECT_EQ(controller.rejected_joint_commands(), 1U);
  EXPECT_FALSE(controller.joint_interface_available());
}

TEST(RobotController, DriverReadyGateBlocksStaleRobotStateAndVelocity) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.update_robot_state(ready_state());
  EXPECT_FALSE(controller.accept_velocity({0.4F, 0, 0}).accepted);
  EXPECT_FALSE(controller.request_mode("stand").accepted);

  controller.set_driver_ready(true);
  ASSERT_TRUE(controller.accept_velocity({0.4F, 0, 0}).accepted);
  controller.set_driver_ready(false);
  controller.update_robot_state(ready_state());
  EXPECT_FALSE(controller.accept_velocity({0.2F, 0, 0}).accepted);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
}

TEST(RobotController, ModeTransitionForcesZeroAndMoveRequiresFreshVelocity) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.set_driver_ready(true);
  controller.update_robot_state(ready_state());
  ASSERT_TRUE(controller.accept_velocity({0.6F, 0, 0}).accepted);

  ASSERT_TRUE(controller.request_mode("stand").accepted);
  EXPECT_TRUE(controller.mode_transition_pending());
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
  EXPECT_FALSE(controller.accept_velocity({0.5F, 0, 0}).accepted);
  controller.complete_mode_transition(false);
  EXPECT_FALSE(controller.mode_transition_pending());
  EXPECT_FALSE(controller.accept_velocity({0.4F, 0, 0}).accepted);

  ASSERT_TRUE(controller.request_mode("move").accepted);
  EXPECT_TRUE(controller.mode_transition_pending());
  controller.complete_mode_transition(true);
  EXPECT_FALSE(controller.mode_transition_pending());
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
  EXPECT_TRUE(controller.accept_velocity({0.3F, 0, 0}).accepted);
}

TEST(RobotController, InvalidateConnectionClearsReadinessAndPendingButKeepsEstop) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.update_robot_state(ready_state());
  EXPECT_FALSE(controller.accept_velocity({0.1F, 0, 0}).accepted);

  controller.set_driver_ready(true);
  controller.update_robot_state(ready_state());
  EXPECT_TRUE(controller.accept_velocity({0.1F, 0, 0}).accepted);
  EXPECT_TRUE(controller.request_mode("stand").accepted);
  EXPECT_TRUE(controller.mode_transition_pending());
  EXPECT_FALSE(controller.accept_velocity({0.1F, 0, 0}).accepted);

  controller.invalidate_connection();
  EXPECT_FALSE(controller.mode_transition_pending());
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
  EXPECT_FALSE(controller.accept_velocity({0.1F, 0, 0}).accepted);
  EXPECT_FALSE(controller.request_mode("stand").accepted);

  controller.trigger_estop();
  controller.invalidate_connection();
  EXPECT_TRUE(controller.emergency_stop_latched());
  EXPECT_FALSE(controller.accept_velocity({0.1F, 0, 0}).accepted);
  EXPECT_TRUE(controller.clear_estop().accepted);
  EXPECT_FALSE(controller.emergency_stop_latched());
  EXPECT_FALSE(controller.accept_velocity({0.1F, 0, 0}).accepted);
}

TEST(RobotController, DisconnectDoesNotRestoreReadinessModeOrVelocity) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  controller.set_driver_ready(true);
  controller.update_robot_state(ready_state());
  ASSERT_TRUE(controller.accept_velocity({0.4F, 0, 0}).accepted);
  ASSERT_TRUE(controller.request_mode("stand").accepted);

  controller.invalidate_connection();
  controller.update_robot_state(ControllerStatus{});
  EXPECT_FALSE(controller.mode_transition_pending());
  EXPECT_FALSE(controller.accept_velocity({0.2F, 0, 0}).accepted);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});

  // Reconnecting with a fully ready status must not restore authority,
  // pending mode, or velocity: only set_driver_ready(true) re-arms motion.
  controller.update_robot_state(ready_state());
  EXPECT_FALSE(controller.mode_transition_pending());
  EXPECT_FALSE(controller.request_mode("stand").accepted);
  EXPECT_FALSE(controller.accept_velocity({0.2F, 0, 0}).accepted);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
}

}  // namespace
}  // namespace hypertron_ros2_bridge
