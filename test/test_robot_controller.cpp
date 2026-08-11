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
  controller.update_robot_state(ready_state());
  EXPECT_EQ(controller.accept_velocity({2.0F, -2.0F, 0.5F}).value,
            (VelocityPayload{1.0F, -1.0F, 0.5F}));
  clock.advance(101ms);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
}

TEST(RobotController, NonFiniteInputForcesZero) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
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
  controller.update_robot_state(ready_state());
  controller.accept_velocity({0.5F, 0.0F, 0.0F});
  EXPECT_TRUE(controller.trigger_estop().accepted);
  EXPECT_FALSE(controller.accept_velocity({0.2F, 0.0F, 0.0F}).accepted);
  EXPECT_TRUE(controller.clear_estop().accepted);
  EXPECT_EQ(controller.velocity_for_tick().value, VelocityPayload{});
}

TEST(RobotController, MapsAllDocumentedModesCaseInsensitively) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
  auto state = ready_state();
  state.sport_status = 0xB102U;
  controller.update_robot_state(state);
  EXPECT_EQ(controller.request_mode(" DAMPING ").mode, 0xA101U);
  EXPECT_EQ(controller.request_mode("Stand").mode, 0xA102U);
  EXPECT_EQ(controller.request_mode("down").mode, 0xA103U);
  EXPECT_EQ(controller.request_mode("MOVE").mode, 0xA104U);
  EXPECT_EQ(controller.request_mode("auto_charge").mode, 0xA105U);
  EXPECT_EQ(controller.request_mode("exit_charge").mode, 0xA106U);
  EXPECT_EQ(controller.request_mode("recovery").mode, 0xA1FFU);
  EXPECT_FALSE(controller.request_mode("run").accepted);
}

TEST(RobotController, AuthorityLossAndFaultForceZero) {
  ManualClock clock;
  RobotController controller({100ms}, clock);
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

}  // namespace
}  // namespace hypertron_ros2_bridge
