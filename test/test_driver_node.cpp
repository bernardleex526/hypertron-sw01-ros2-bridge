// ROS graph integration tests for HypertronDriverNode. The node runs over the
// production DirectDriverRuntime driven by the pure, vendor-free fakes from
// this directory (FakeAstrallSdk / FakeNetworkPreflight), so no vendor SDK and
// no network are exercised. Every temporal wait is bounded by a 2 s
// wait_until helper; no test asserts on a bare sleep.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/astrall_sdk.hpp"
#include "hypertron_ros2_bridge/driver_node.hpp"
#include "hypertron_ros2_bridge/lidar_stream.hpp"
#include "hypertron_ros2_bridge/msg/robot_state.hpp"
#include "fake_astrall_sdk.hpp"
#include "fake_lidar_datagram_source.hpp"
#include "fake_network_preflight.hpp"

namespace {

namespace test = hypertron_ros2_bridge::test;
using hypertron_ros2_bridge::HypertronDriverNode;
using hypertron_ros2_bridge::IAstrallSdk;
using hypertron_ros2_bridge::INetworkPreflight;
using hypertron_ros2_bridge::ImuSample;
using hypertron_ros2_bridge::msg::RobotState;
using hypertron_ros2_bridge::SportSample;
using hypertron_ros2_bridge::Velocity;
using test::FakeAstrallSdk;
using test::FakeLidarDatagramSource;
using test::FakeNetworkPreflight;

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

// A complete packed point-cloud packet that reassembles on its own: one
// packet with total==index_count==1 carrying `points` (pos_num points).
std::vector<std::uint8_t> MakeWholeCloudPacket(
    std::uint64_t ts, std::uint32_t total, std::uint32_t index,
    std::uint16_t pos_num, const std::vector<std::array<int, 4>>& points) {
  const std::size_t pos_at = 20U;  // packed header; tail after pos_num points
  std::vector<std::uint8_t> b(pos_at + 28U * pos_num + 2U, 0);
  PutU16(b, 0, 0xAA55U);
  PutU64(b, 2, ts);
  PutU32(b, 10, total);
  PutU32(b, 14, index);
  PutU16(b, 18, pos_num);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const std::size_t off = pos_at + i * 28U;
    PutU32(b, off, static_cast<std::uint32_t>(static_cast<std::int32_t>(points[i][0])));
    PutU32(b, off + 4, static_cast<std::uint32_t>(static_cast<std::int32_t>(points[i][1])));
    PutU32(b, off + 8, static_cast<std::uint32_t>(static_cast<std::int32_t>(points[i][2])));
    PutU32(b, off + 12, static_cast<std::uint32_t>(points[i][3]));
  }
  PutU16(b, b.size() - 2U, 0xFF00U);
  return b;
}

// A packed 68-byte odometry datagram.
std::vector<std::uint8_t> MakeOdometryPacket(
    std::uint64_t ts, std::int64_t x, std::int64_t y, std::int64_t z,
    std::int64_t qx, std::int64_t qy, std::int64_t qz, std::int64_t qw) {
  std::vector<std::uint8_t> b(68U, 0);
  PutU16(b, 0, 0xAA55U);
  PutU64(b, 2, ts);
  PutU64(b, 10, static_cast<std::uint64_t>(x));
  PutU64(b, 18, static_cast<std::uint64_t>(y));
  PutU64(b, 26, static_cast<std::uint64_t>(z));
  PutU64(b, 34, static_cast<std::uint64_t>(qx));
  PutU64(b, 42, static_cast<std::uint64_t>(qy));
  PutU64(b, 50, static_cast<std::uint64_t>(qz));
  PutU64(b, 58, static_cast<std::uint64_t>(qw));
  PutU16(b, 66, 0xFF00U);
  return b;
}

constexpr auto kWaitTimeout = std::chrono::seconds(2);

// Fully defined here so the fakes can be constructed with the parameter
// overrides the runtime needs to run quickly in tests. These field names and
// defaults mirror driver_node.cpp's parameter declaration.
rclcpp::NodeOptions test_node_options() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("timing.heartbeat_period_ms", 5),
      rclcpp::Parameter("timing.motion_refresh_period_ms", 5),
      rclcpp::Parameter("timing.state_poll_period_ms", 5),
      rclcpp::Parameter("timing.reconnect_initial_delay_ms", 10),
      rclcpp::Parameter("timing.reconnect_max_delay_ms", 20),
      rclcpp::Parameter("timing.mode_timeout_ms", 100),
      rclcpp::Parameter("sdk.call_timeout_ms", 500),
      rclcpp::Parameter("sdk.heartbeat_call_timeout_ms", 100),
      rclcpp::Parameter("sdk.init_timeout_ms", 1000),
  };
  return options;
}

// Bounded spin loop that evaluates pred() at least once per iteration up to
// timeout. Returns true as soon as pred() returns true.
template <typename Pred>
bool wait_until(Pred&& pred, std::chrono::milliseconds timeout = kWaitTimeout,
                std::chrono::milliseconds step = std::chrono::milliseconds(5)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  do {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(step);
  } while (std::chrono::steady_clock::now() < deadline);
  return pred();
}

// A live-connected driver node plus raw access to its injected fakes.
struct Harness {
  std::shared_ptr<HypertronDriverNode> node;
  FakeAstrallSdk* sdk = nullptr;
  FakeNetworkPreflight* preflight = nullptr;

  // The driver node owns the fakes; sdk/preflight point into it via raw
  // pointers captured before ownership transfers. They stay valid for the
  // lifetime of `node`.
  static Harness make(bool connected) {
    Harness h;
    auto s = std::make_unique<FakeAstrallSdk>();
    auto p = std::make_unique<FakeNetworkPreflight>();
    h.sdk = s.get();
    h.preflight = p.get();
    if (connected) {
      // Drive-ready armed once the first snapshot polls: link+authority and
      // the all-terrain Move sport status the motion gate requires.
      s->set_linked(true);
      s->set_authority(true);
      s->set_sport_status(0xB104U);
    }
    p->next_decision.ready = true;
    p->next_decision.message = "fake preflight ready";
    h.node = std::make_shared<HypertronDriverNode>(
        test_node_options(), std::move(s), std::move(p));
    return h;
  }

  // As `make`, but also injects programmatic LiDAR datagram sources so tests
  // can push point-cloud / odometry datagrams directly into the receivers.
  static Harness make_with_lidar(
      std::unique_ptr<FakeLidarDatagramSource> point_cloud,
      std::unique_ptr<FakeLidarDatagramSource> odometry,
      bool connected) {
    Harness h;
    auto s = std::make_unique<FakeAstrallSdk>();
    auto p = std::make_unique<FakeNetworkPreflight>();
    h.sdk = s.get();
    h.preflight = p.get();
    if (connected) {
      s->set_linked(true);
      s->set_authority(true);
      s->set_sport_status(0xB104U);
    }
    p->next_decision.ready = true;
    p->next_decision.message = "fake preflight ready";
    h.node = std::make_shared<HypertronDriverNode>(
        test_node_options(), std::move(s), std::move(p), std::move(point_cloud),
        std::move(odometry));
    return h;
  }
};

// Drives a "connected" session until the driver readiness gate is armed:
// emits an SDK status report (sets the runtime link/authority caches) and then
// waits for several heartbeat ticks that also advance state polls, so the
// first sdk_linked snapshot arms the motion gate.
void arm_connected(Harness& h,
                   rclcpp::executors::SingleThreadedExecutor& executor) {
  // Wait for the worker to enter a connected session (init+subscribe done).
  ASSERT_TRUE(wait_until([&] { return h.sdk->was_called("heartbeat"); }))
      << "worker never connected (no heartbeat recorded)";
  h.sdk->emit_status(true, true);
  // Advance several heartbeat / state-poll ticks so a live snapshot is polled.
  ASSERT_TRUE(wait_until([&] {
    return h.sdk->call_count("heartbeat") >= 5;
  })) << "state polls did not advance after status report";
}

// Returns the first recorded move() velocity with the given clamped values,
// or nullopt if none matched within timeout. Spins the executor so the cmd_vel
// subscription callback is delivered to the driver node on this thread.
std::optional<Velocity> wait_for_move(
    Harness& h, rclcpp::executors::SingleThreadedExecutor& executor,
    std::int32_t vx, std::int32_t vy, std::int32_t vyaw,
    std::chrono::milliseconds timeout = kWaitTimeout) {
  std::optional<Velocity> out;
  wait_until(
      [&] {
        executor.spin_some();
        for (const auto& call : h.sdk->calls()) {
          if (call.method == "move" &&
              static_cast<int>(call.velocity.vx) == vx &&
              static_cast<int>(call.velocity.vy) == vy &&
              static_cast<int>(call.velocity.vyaw) == vyaw) {
            out = call.velocity;
            return true;
          }
        }
        return false;
      },
      timeout);
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

class DriverNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestCase() { /* rclcpp initialized in main() */ }
  void TearDown() override {
    // Ensure the executor is out of scope before the node so the worker is
    // stopped cleanly.
  }
};

// ---------------------------------------------------------------------------
// 1. Graph interface exposure
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, NodeExposesExpectedInterfaces) {
  Harness h = Harness::make(/*connected=*/true);
  auto graph = h.node->get_node_graph_interface();
  EXPECT_EQ(graph->count_subscribers("/cmd_vel"), 1U);
  EXPECT_EQ(graph->count_subscribers("/robot_mode"), 1U);
  EXPECT_EQ(graph->count_subscribers("/joint_commands"), 1U);
  EXPECT_EQ(graph->count_publishers("/imu/data"), 1U);
  EXPECT_EQ(graph->count_publishers("/odom"), 1U);
  EXPECT_EQ(graph->count_publishers("/robot_state"), 1U);

  // Service counts are exposed via get_service_names_and_types().
  const auto services = graph->get_service_names_and_types();
  EXPECT_EQ(services.count("/emergency_stop"), 1U);
  EXPECT_EQ(services.count("/control_authority"), 1U);
}

// ---------------------------------------------------------------------------
// 2. Joint commands are rejected and counted
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, JointCommandsAreRejectedAndCounted) {
  Harness h = Harness::make(/*connected=*/true);
  auto aux = std::make_shared<rclcpp::Node>("aux_joint");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);
  executor.add_node(aux);

  std::mutex state_mutex;
  std::optional<RobotState> received;
  auto state_sub = aux->create_subscription<RobotState>(
      "/robot_state", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](const RobotState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        received = *msg;
      });
  auto joint_pub = aux->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_commands", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // Let graph discovery / callback registration settle.
  executor.spin_some(std::chrono::milliseconds(50));

  auto joint = sensor_msgs::msg::JointState();
  joint.name.push_back("wheel_0");
  joint.position.push_back(0.5);
  joint_pub->publish(joint);

  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    std::lock_guard<std::mutex> lock(state_mutex);
    return received.has_value() && received->rejected_joint_commands >= 1;
  })) << "robot_state with a rejected joint command was never received";

  std::lock_guard<std::mutex> lock(state_mutex);
  ASSERT_TRUE(received.has_value());
  EXPECT_GE(received->rejected_joint_commands, 1U);
  EXPECT_FALSE(received->joint_interface_available);
  EXPECT_FALSE(received->camera_available);
  EXPECT_FALSE(received->ssh_connected);
  EXPECT_FALSE(received->agent_connected);
}

// ---------------------------------------------------------------------------
// 3. cmd_vel is clamped and dispatched
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, CmdVelIsClampedAndDispatched) {
  Harness h = Harness::make(/*connected=*/true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);

  auto aux = std::make_shared<rclcpp::Node>("aux_cmdvel");
  executor.add_node(aux);

  // Confirm the driver readiness gate is truly armed before publishing a
  // velocity: the gate arms only after a live state poll mirrors the Move
  // sport status (0xB104) while the SDK link is reported. on_state republishes
  // robot_state with the snapshot's sport_status, so observing that here proves
  // the gate has seen the armed state.
  std::mutex gate_mutex;
  bool gate_armed = false;
  auto gate_sub = aux->create_subscription<RobotState>(
      "/robot_state", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](const RobotState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(gate_mutex);
        if (msg->sdk_linked && msg->sport_status == 0xB104U) {
          gate_armed = true;
        }
      });
  arm_connected(h, executor);
  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    std::lock_guard<std::mutex> lock(gate_mutex);
    return gate_armed;
  })) << "driver readiness gate never armed";

  auto cmd_pub = aux->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto mode_pub = aux->create_publisher<std_msgs::msg::String>(
      "/robot_mode", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // The velocity safety gate only opens once the robot is confirmed in the
  // all-terrain Move mode: a "move" /robot_mode command starts a mode
  // transition that the worker settles as soon as the polled sport status is
  // the Move state (0xB104, already set in the fake snapshot). Wait for the
  // SDK to have been asked to enter Move before dispatching velocity.
  auto mode_msg = std_msgs::msg::String();
  mode_msg.data = "move";
  mode_pub->publish(mode_msg);
  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    for (const auto& call : h.sdk->calls()) {
      if (call.method == "set_mode" && call.mode == 0xA104U) {
        return true;
      }
    }
    return false;
  })) << "move mode was never requested from the SDK";

  // Dispatch cmd_vel, re-publishing in case the first attempt lands while the
  // mode transition is still closing the gate.
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 2.5;
  twist.linear.y = -3.0;
  twist.angular.z = 0.5;
  const auto deadline = std::chrono::steady_clock::now() + kWaitTimeout;
  bool seen = false;
  do {
    cmd_pub->publish(twist);
    if (wait_for_move(h, executor, 1, -1, 0,
                      std::chrono::milliseconds(25))) {
      seen = true;
      break;
    }
  } while (std::chrono::steady_clock::now() < deadline);
  ASSERT_TRUE(seen) << "clamped velocity was not dispatched";

  auto move = wait_for_move(h, executor, 1, -1, 0);
  ASSERT_TRUE(move.has_value()) << "clamped velocity was not dispatched";
  EXPECT_FLOAT_EQ(move->vx, 1.0f);
  EXPECT_FLOAT_EQ(move->vy, -1.0f);
  EXPECT_FLOAT_EQ(move->vyaw, 0.5f);
}

// ---------------------------------------------------------------------------
// 3b. /cmd_vel with a non-finite (NaN/Inf) component is rejected, never
// submitted. clamp_unit() alone would silently turn NaN into +1.0f and slip
// past the controller's non-finite rejection, so the callback must refuse the
// whole twist before clamping. The rejection must be isolated: a valid twist
// published afterwards still reaches the SDK.
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, CmdVelNaNIsRejectedNotClamped) {
  Harness h = Harness::make(/*connected=*/true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);
  auto aux = std::make_shared<rclcpp::Node>("aux_cmdvel_nan");
  executor.add_node(aux);

  arm_connected(h, executor);

  auto cmd_pub = aux->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  auto mode_pub = aux->create_publisher<std_msgs::msg::String>(
      "/robot_mode", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // Arm the velocity gate by requesting Move mode.
  auto mode_msg = std_msgs::msg::String();
  mode_msg.data = "move";
  mode_pub->publish(mode_msg);
  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    for (const auto& call : h.sdk->calls()) {
      if (call.method == "set_mode" && call.mode == 0xA104U) {
        return true;
      }
    }
    return false;
  })) << "move mode was never requested from the SDK";

  auto publish_nonfinite = [&](double x, double y, double z) {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = x;
    twist.linear.y = y;
    twist.angular.z = z;
    cmd_pub->publish(twist);
  };

  // Publish NaN then ±Inf twists; if the callback were clamp-only the NaN
  // linear.x would be dispatched as +1.0f. Assert no move appeared at all in
  // the window immediately after each non-finite publication.
#if __cpp_lib_hypot >= 201603L
  const double nan = std::nan("");
#else
  const double nan = std::numeric_limits<double>::quiet_NaN();
#endif
  const auto baseline = h.sdk->call_count("move");
  publish_nonfinite(nan, 0.0, 0.0);  // NaN linear.x
  publish_nonfinite(0.0, nan, 0.0);  // NaN linear.y
  publish_nonfinite(0.0, 0.0, nan);  // NaN angular.z
  publish_nonfinite(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  publish_nonfinite(-std::numeric_limits<double>::infinity(), 0.0, 0.0);
  publish_nonfinite(0.0, 0.0, std::numeric_limits<double>::infinity());

  // Allow any mis-dispatched move to appear, then require none did.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  executor.spin_some();
  // No new move is allowed to come from the non-finite /cmd_vel messages;
  // any move before them must be an idle/estop artifact, so require the count
  // to be exactly the baseline after absorbing in-flight executor delivery.
  EXPECT_EQ(h.sdk->call_count("move"), baseline)
      << "non-finite /cmd_vel was clamped and dispatched instead of rejected";

  // Prove the rejection was isolated: a valid twist is still dispatched.
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = 0.75;
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;
  const auto deadline = std::chrono::steady_clock::now() + kWaitTimeout;
  bool seen = false;
  do {
    cmd_pub->publish(twist);
    if (wait_for_move(h, executor, 0, 0, 0, std::chrono::milliseconds(25))) {
      seen = true;
      break;
    }
  } while (std::chrono::steady_clock::now() < deadline);
  EXPECT_TRUE(seen) << "valid /cmd_vel after a non-finite rejection was lost";
}

// ---------------------------------------------------------------------------
// 4. IMU mapping with covariance; non-finite samples are dropped
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, ImuMappingPublishesImuWithCovariance) {
  Harness h = Harness::make(/*connected=*/true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);

  auto aux = std::make_shared<rclcpp::Node>("aux_imu");
  executor.add_node(aux);
  std::mutex imu_mutex;
  std::size_t imu_count = 0;
  std::optional<sensor_msgs::msg::Imu> last_imu;
  auto imu_sub = aux->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_mutex);
        last_imu = *msg;
        ++imu_count;
      });

  // The IMU callback is only installed once the session subscribed.
  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    return h.sdk->was_called("subscribe_imu");
  })) << "imu subscription never installed";

  ImuSample sample;
  sample.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
  sample.pitch = 0.1f;
  sample.roll = 0.2f;
  sample.yaw = 0.3f;
  sample.odom_x = 1.0f;
  sample.odom_y = 2.0f;
  sample.accelerometer = {0.1f, 0.2f, 9.81f};
  sample.gyroscope = {0.01f, 0.02f, 0.03f};
  h.sdk->emit_imu(sample);

  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_count >= 1;
  })) << "valid IMU sample was not published";

  sensor_msgs::msg::Imu imu;
  {
    std::lock_guard<std::mutex> lock(imu_mutex);
    imu = *last_imu;
  }
  EXPECT_FLOAT_EQ(imu.orientation.x, 1.0f);
  EXPECT_FLOAT_EQ(imu.orientation.y, 0.0f);
  EXPECT_FLOAT_EQ(imu.orientation.z, 0.0f);
  EXPECT_FLOAT_EQ(imu.orientation.w, 0.0f);
  EXPECT_FLOAT_EQ(imu.orientation_covariance[0], 0.02);
  EXPECT_FLOAT_EQ(imu.orientation_covariance[4], 0.02);
  EXPECT_FLOAT_EQ(imu.orientation_covariance[8], 0.05);
  EXPECT_FLOAT_EQ(imu.linear_acceleration.x, 0.1f);

  // A non-finite sample must be dropped (count unchanged).
  ImuSample bad = sample;
  bad.gyroscope[0] = std::numeric_limits<float>::quiet_NaN();
  h.sdk->emit_imu(bad);
  executor.spin_some(std::chrono::milliseconds(50));
  std::lock_guard<std::mutex> lock(imu_mutex);
  EXPECT_EQ(imu_count, 1U) << "non-finite IMU sample was not dropped";
}

// ---------------------------------------------------------------------------
// 5. Emergency stop service answers truthfully
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, EmergencyStopServiceAnswersTruthfully) {
  // (a) Disconnected / not armed: the latched request is not confirmed.
  {
    Harness h = Harness::make(/*connected=*/false);
    auto aux = std::make_shared<rclcpp::Node>("aux_estop_a");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(h.node);
    executor.add_node(aux);
    executor.spin_some(std::chrono::milliseconds(50));

    auto client =
        aux->create_client<std_srvs::srv::SetBool>("/emergency_stop");
    ASSERT_TRUE(client->wait_for_service(std::chrono::milliseconds(500)));
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    auto future = client->async_send_request(req);
    auto result = executor.spin_until_future_complete(future, kWaitTimeout);
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_TRUE(future.get()->success == false)
        << "estop without a live SDK link must not succeed";
  }

  // (b) Connected and armed: the software estop is dispatched.
  {
    Harness h = Harness::make(/*connected=*/true);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(h.node);
    arm_connected(h, executor);

    auto aux = std::make_shared<rclcpp::Node>("aux_estop_b");
    executor.add_node(aux);
    auto client =
        aux->create_client<std_srvs::srv::SetBool>("/emergency_stop");
    ASSERT_TRUE(client->wait_for_service(std::chrono::milliseconds(500)));
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    auto future = client->async_send_request(req);
    auto result = executor.spin_until_future_complete(future, kWaitTimeout);
    ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);

    // The worker must have issued zero velocity and damping mode.
    EXPECT_TRUE(wait_until([&] { return h.sdk->was_called("move"); }))
        << "no move was dispatched for the estop";
    EXPECT_TRUE(wait_until([&] {
      for (const auto& call : h.sdk->calls()) {
        if (call.method == "move" && call.velocity.vx == 0.0f &&
            call.velocity.vy == 0.0f && call.velocity.vyaw == 0.0f) {
          return true;
        }
      }
      return false;
    })) << "estop did not zero velocity";
    EXPECT_TRUE(wait_until([&] {
      for (const auto& call : h.sdk->calls()) {
        if (call.method == "set_mode" && call.mode == 0xA101U) {
          return true;
        }
      }
      return false;
    })) << "estop did not request damping mode";
  }
}

// ---------------------------------------------------------------------------
// 6. RobotState direct-drive fields
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, RobotStateDirectModeFields) {
  Harness h = Harness::make(/*connected=*/true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);

  auto aux = std::make_shared<rclcpp::Node>("aux_state");
  executor.add_node(aux);
  std::mutex state_mutex;
  std::optional<RobotState> received;
  auto state_sub = aux->create_subscription<RobotState>(
      "/robot_state", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      [&](const RobotState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        received = *msg;
      });

  // Link/authority are owned by on_status() rather than by state snapshots;
  // emit the SDK status callback before expecting sdk_linked to be true.
  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    return h.sdk->was_called("heartbeat");
  }));
  h.sdk->emit_status(true, true);

  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    std::lock_guard<std::mutex> lock(state_mutex);
    return received.has_value() && received->sdk_linked;
  })) << "robot_state with sdk_linked never received";

  std::lock_guard<std::mutex> lock(state_mutex);
  ASSERT_TRUE(received.has_value());
  EXPECT_TRUE(received->sdk_linked);
  EXPECT_FALSE(received->ssh_connected);
  EXPECT_FALSE(received->agent_connected);
  EXPECT_FALSE(received->joint_interface_available);
  EXPECT_FALSE(received->camera_available);
  EXPECT_FALSE(received->odometry_scale_verified);
}

// ---------------------------------------------------------------------------
// 7. TF broadcaster requires a verified odometry scale
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, TfBroadcasterRequiresVerifiedScale) {
  // (a) publish_tf true but scale unverified: no /tf is broadcast.
  {
    rclcpp::NodeOptions opts = test_node_options();
    opts.parameter_overrides().push_back(rclcpp::Parameter("odom.publish_tf", true));
    opts.parameter_overrides().push_back(
        rclcpp::Parameter("odometry.scale_verified", false));
    auto s = std::make_unique<FakeAstrallSdk>();
    auto p = std::make_unique<FakeNetworkPreflight>();
    FakeAstrallSdk* sdk = s.get();
    s->set_linked(true);
    s->set_authority(true);
    s->set_sport_status(0xB104U);
    p->next_decision.ready = true;
    p->next_decision.message = "ok";
    auto node =
        std::make_shared<HypertronDriverNode>(opts, std::move(s), std::move(p));

    auto aux = std::make_shared<rclcpp::Node>("aux_tf_a");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(aux);
    std::mutex tf_mutex;
    std::size_t tf_count = 0;
    auto tf_sub = aux->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [&](const tf2_msgs::msg::TFMessage::SharedPtr) {
          std::lock_guard<std::mutex> lock(tf_mutex);
          ++tf_count;
        });
    executor.spin_some(std::chrono::milliseconds(100));
    // Emit IMU so a pose exists; still no broadcaster should be active.
    if (wait_until([&] { return sdk->was_called("subscribe_imu"); })) {
      ImuSample sample;
      sample.odom_x = 1.0f;
      sample.odom_y = 2.0f;
      sample.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
      sdk->emit_imu(sample);
    }
    executor.spin_some(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> lock(tf_mutex);
    EXPECT_EQ(tf_count, 0U) << "/tf was broadcast without a verified scale";
  }

  // (b) Both flags true: an IMU pose produces an odom->base transform.
  {
    rclcpp::NodeOptions opts = test_node_options();
    opts.parameter_overrides().push_back(rclcpp::Parameter("odom.publish_tf", true));
    opts.parameter_overrides().push_back(
        rclcpp::Parameter("odometry.scale_verified", true));
    auto s = std::make_unique<FakeAstrallSdk>();
    auto p = std::make_unique<FakeNetworkPreflight>();
    FakeAstrallSdk* sdk = s.get();
    s->set_linked(true);
    s->set_authority(true);
    s->set_sport_status(0xB104U);
    p->next_decision.ready = true;
    p->next_decision.message = "ok";
    auto node =
        std::make_shared<HypertronDriverNode>(opts, std::move(s), std::move(p));

    auto aux = std::make_shared<rclcpp::Node>("aux_tf_b");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(aux);
    std::mutex tf_mutex;
    std::optional<tf2_msgs::msg::TFMessage> tf_msg;
    auto tf_sub = aux->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [&](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(tf_mutex);
          tf_msg = *msg;
        });

    ASSERT_TRUE(wait_until([&] { return sdk->was_called("subscribe_imu"); }))
        << "imu subscription never installed";
    ImuSample sample;
    sample.odom_x = 1.0f;
    sample.odom_y = 2.0f;
    sample.quaternion = {1.0f, 0.0f, 0.0f, 0.0f};
    sdk->emit_imu(sample);

    ASSERT_TRUE(wait_until([&] {
      executor.spin_some();
      std::lock_guard<std::mutex> lock(tf_mutex);
      return tf_msg.has_value();
    })) << "/tf was never broadcast";

    std::lock_guard<std::mutex> lock(tf_mutex);
    ASSERT_TRUE(tf_msg.has_value());
    ASSERT_FALSE(tf_msg->transforms.empty());
    EXPECT_EQ(tf_msg->transforms[0].header.frame_id, "odom");
    EXPECT_EQ(tf_msg->transforms[0].child_frame_id, "base_link");
  }
}

// ---------------------------------------------------------------------------
// 8. Control authority service forwards the result
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, ControlAuthorityServiceForwardsResult) {
  Harness h = Harness::make(/*connected=*/true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);
  arm_connected(h, executor);

  auto aux = std::make_shared<rclcpp::Node>("aux_authority");
  executor.add_node(aux);
  auto client =
      aux->create_client<std_srvs::srv::SetBool>("/control_authority");
  ASSERT_TRUE(client->wait_for_service(std::chrono::milliseconds(500)));

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = true;
  auto future = client->async_send_request(req);
  auto result = executor.spin_until_future_complete(future, kWaitTimeout);
  ASSERT_EQ(result, rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);

  EXPECT_TRUE(wait_until([&] {
    for (const auto& call : h.sdk->calls()) {
      if (call.method == "request_authority" && call.authority_sdk) {
        return true;
      }
    }
    return false;
  })) << "control authority was not forwarded to the SDK";
}

// ---------------------------------------------------------------------------
// 9. LiDAR bypass stream (UDP 6100/6101)
// ---------------------------------------------------------------------------
TEST_F(DriverNodeTest, LidarPublishersExistByDefault) {
  Harness h = Harness::make(/*connected=*/true);
  auto graph = h.node->get_node_graph_interface();
  EXPECT_EQ(graph->count_publishers("/points"), 1U);
  EXPECT_EQ(graph->count_publishers("/odom_lidar"), 1U);
  // The runtime must subscribe LIDAR once per connected session.
  EXPECT_TRUE(wait_until([&] { return h.sdk->call_count("subscribe_lidar") >= 1; }));
}

TEST_F(DriverNodeTest, SdkSubscribesLidarPerSession) {
  Harness h = Harness::make(/*connected=*/true);
  auto aux = std::make_shared<rclcpp::Node>("aux_lidar_sub");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);
  executor.add_node(aux);
  executor.spin_some(std::chrono::milliseconds(20));
  // The default runtime configuration enables the LIDAR stream, so a
  // connected session always subscribes LIDAR.
  ASSERT_TRUE(wait_until([&] { return h.sdk->was_called("heartbeat"); }));
  EXPECT_TRUE(wait_until([&] { return h.sdk->call_count("subscribe_lidar") >= 1; }));
}

TEST_F(DriverNodeTest, PointCloudPacketPublishesPoints) {
  // Inject a fake point-cloud source (odometry source unused here).
  auto pc = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* pc_ptr = pc.get();
  auto odom = std::make_unique<FakeLidarDatagramSource>();
  Harness h = Harness::make_with_lidar(std::move(pc), std::move(odom),
                                       /*connected=*/true);
  auto aux = std::make_shared<rclcpp::Node>("aux_points");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);
  executor.add_node(aux);

  std::mutex points_mutex;
  std::size_t points_count = 0;
  std::optional<sensor_msgs::msg::PointCloud2> last_cloud;
  auto points_sub = aux->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points", rclcpp::SensorDataQoS(),
      [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(points_mutex);
        last_cloud = *msg;
        ++points_count;
      });

  // The assembler treats `total` as the frame point count and reassembles a
  // frame from indexed packets whose point sets exactly cover it. Push two
  // single-point packets (indices 0 and 1, total=2): the frame publishes as
  // soon as index 1 arrives.
  // Point 0: x=1000*1e-3=1.0m, y=2000*1e-3=2.0m, z=3000*1e-3=3.0m, rgba raw.
  // Point 1: x=4000*1e-3=4.0m, y=5000*1e-3=5.0m, z=6000*1e-3=6.0m.
  std::vector<std::array<int, 4>> p0 = {{1000, 2000, 3000, 0x00FF0000}};
  std::vector<std::array<int, 4>> p1 = {{4000, 5000, 6000, 0x00112233}};
  auto packet0 = MakeWholeCloudPacket(123456ULL, 2, 0, 1, p0);
  auto packet1 = MakeWholeCloudPacket(123456ULL, 2, 1, 1, p1);
  ASSERT_TRUE(pc_ptr->push(std::move(packet0)));
  ASSERT_TRUE(pc_ptr->push(std::move(packet1)));

  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    std::lock_guard<std::mutex> lock(points_mutex);
    return points_count >= 1;
  })) << "point cloud was never published";

  sensor_msgs::msg::PointCloud2 cloud;
  {
    std::lock_guard<std::mutex> lock(points_mutex);
    cloud = *last_cloud;
  }
  EXPECT_EQ(cloud.header.frame_id, "lidar");
  EXPECT_EQ(cloud.height, 1U);
  EXPECT_EQ(cloud.width, 2U);
  EXPECT_TRUE(cloud.is_dense);
  EXPECT_EQ(cloud.point_step, 16U);
  ASSERT_EQ(cloud.fields.size(), 4U);
  EXPECT_EQ(cloud.fields[0].name, "x");
  EXPECT_EQ(cloud.fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(cloud.fields[0].offset, 0U);
  EXPECT_EQ(cloud.fields[1].name, "y");
  EXPECT_EQ(cloud.fields[1].offset, 4U);
  EXPECT_EQ(cloud.fields[2].name, "z");
  EXPECT_EQ(cloud.fields[2].offset, 8U);
  EXPECT_EQ(cloud.fields[3].name, "rgba");
  EXPECT_EQ(cloud.fields[3].datatype, sensor_msgs::msg::PointField::UINT32);
  EXPECT_EQ(cloud.fields[3].offset, 12U);

  ASSERT_EQ(cloud.data.size(), 32U);  // 2 points * 16-byte stride
  float x, y, z;
  std::uint32_t rgba;
  std::memcpy(&x, cloud.data.data(), sizeof(float));
  std::memcpy(&y, cloud.data.data() + 4, sizeof(float));
  std::memcpy(&z, cloud.data.data() + 8, sizeof(float));
  std::memcpy(&rgba, cloud.data.data() + 12, sizeof(rgba));
  EXPECT_FLOAT_EQ(x, 1.0f);
  EXPECT_FLOAT_EQ(y, 2.0f);
  EXPECT_FLOAT_EQ(z, 3.0f);
  EXPECT_EQ(rgba, 0x00FF0000U);
  std::memcpy(&x, cloud.data.data() + 16, sizeof(float));
  std::memcpy(&y, cloud.data.data() + 20, sizeof(float));
  EXPECT_FLOAT_EQ(x, 4.0f);
  EXPECT_FLOAT_EQ(y, 5.0f);
}

TEST_F(DriverNodeTest, OdometryPacketPublishesOdomLidar) {
  auto pc = std::make_unique<FakeLidarDatagramSource>();
  auto odom = std::make_unique<FakeLidarDatagramSource>();
  FakeLidarDatagramSource* odom_ptr = odom.get();
  Harness h = Harness::make_with_lidar(std::move(pc), std::move(odom),
                                       /*connected=*/true);
  auto aux = std::make_shared<rclcpp::Node>("aux_odom_lidar");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(h.node);
  executor.add_node(aux);

  std::mutex odom_mutex;
  std::size_t odom_count = 0;
  std::optional<nav_msgs::msg::Odometry> last_odom;
  auto odom_sub = aux->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_lidar", rclcpp::SensorDataQoS(),
      [&](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex);
        last_odom = *msg;
        ++odom_count;
      });

  // x=1000000*1e-6=1.0m..., q=(0,0,0,1000000)/1e6 -> (0,0,0,1) normalized.
  auto packet = MakeOdometryPacket(9ULL, 1000000, 2000000, 3000000,
                                   0, 0, 0, 1000000);
  ASSERT_TRUE(odom_ptr->push(std::move(packet)));

  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    std::lock_guard<std::mutex> lock(odom_mutex);
    return odom_count >= 1;
  })) << "lidar odometry was never published";

  nav_msgs::msg::Odometry odom_msg;
  {
    std::lock_guard<std::mutex> lock(odom_mutex);
    odom_msg = *last_odom;
  }
  EXPECT_EQ(odom_msg.header.frame_id, "odom");
  EXPECT_EQ(odom_msg.child_frame_id, "lidar");
  EXPECT_DOUBLE_EQ(odom_msg.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(odom_msg.pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(odom_msg.pose.pose.position.z, 3.0);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(odom_msg.pose.pose.orientation.w, 1.0, 1e-9);
  // Pose covariance diagonal from the configured odom_lidar parameter.
  EXPECT_DOUBLE_EQ(odom_msg.pose.covariance[0], 0.05);
  EXPECT_DOUBLE_EQ(odom_msg.pose.covariance[7], 0.05);
  EXPECT_DOUBLE_EQ(odom_msg.pose.covariance[14], 0.05);
  EXPECT_DOUBLE_EQ(odom_msg.pose.covariance[21], 0.01);
  EXPECT_DOUBLE_EQ(odom_msg.pose.covariance[28], 0.01);
  EXPECT_DOUBLE_EQ(odom_msg.pose.covariance[35], 0.01);
  // No twist is reported: all-zero default.
  EXPECT_EQ(odom_msg.twist.twist.linear.x, 0.0);
}

TEST_F(DriverNodeTest, LidarDisabledPublishesNothing) {
  rclcpp::NodeOptions opts = test_node_options();
  opts.parameter_overrides().push_back(
      rclcpp::Parameter("subscriptions.lidar_enabled", false));
  auto s = std::make_unique<FakeAstrallSdk>();
  auto p = std::make_unique<FakeNetworkPreflight>();
  FakeAstrallSdk* sdk = s.get();
  p->next_decision.ready = true;
  p->next_decision.message = "ok";
  auto node =
      std::make_shared<HypertronDriverNode>(opts, std::move(s), std::move(p));

  auto graph = node->get_node_graph_interface();
  EXPECT_EQ(graph->count_publishers("/points"), 0U);
  EXPECT_EQ(graph->count_publishers("/odom_lidar"), 0U);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto aux = std::make_shared<rclcpp::Node>("aux_lidar_disabled");
  executor.add_node(aux);
  executor.spin_some(std::chrono::milliseconds(50));
  // The runtime never subscribes LIDAR when disabled; a connected session
  // still establishes (imu/sport only).
  ASSERT_TRUE(wait_until([&] { return sdk->was_called("heartbeat"); }));
  EXPECT_EQ(sdk->call_count("subscribe_lidar"), 0U);
}

// ---------------------------------------------------------------------------
// F5: invalid node parameters make the node construction throw
// rclcpp::exceptions::InvalidParametersException (so main exits non-zero)
// rather than silently clamping / wrapping / allocating unbounded buffers.
// ---------------------------------------------------------------------------

// Builds a NodeOptions from the valid test defaults plus the given overrides.
template <typename... Overrides>
rclcpp::NodeOptions options_with(Overrides&&... overrides) {
  rclcpp::NodeOptions opts = test_node_options();
  (opts.parameter_overrides().push_back(std::forward<Overrides>(overrides)), ...);
  return opts;
}

TEST_F(DriverNodeTest, F5_InvalidParametersThrowInvalidParametersException) {
  using rclcpp::exceptions::InvalidParametersException;
  auto try_construct = [](rclcpp::NodeOptions opts) {
    // Validation runs before any receiver/runtime is built, so no fakes/sdk
    // are reached; nullptr is fine because the throw happens first.
    (void)std::make_shared<HypertronDriverNode>(opts);
  };

  // max_parallel_frames outside [1, 64].
  EXPECT_THROW(try_construct(options_with(
                   rclcpp::Parameter("lidar.max_parallel_frames", -1))),
               InvalidParametersException);
  // UDP port outside [1, 65535].
  EXPECT_THROW(try_construct(options_with(
                   rclcpp::Parameter("lidar.point_cloud_port", 70000))),
               InvalidParametersException);
  // reconnect_initial_delay_ms > reconnect_max_delay_ms.
  EXPECT_THROW(try_construct(options_with(
                   rclcpp::Parameter("timing.reconnect_initial_delay_ms", 50),
                   rclcpp::Parameter("timing.reconnect_max_delay_ms", 10))),
               InvalidParametersException);
  // IMU subscription frequency is not one of {0,1,25,50,125,250}.
  EXPECT_THROW(try_construct(options_with(
                   rclcpp::Parameter("subscriptions.imu_frequency_hz", 33))),
               InvalidParametersException);
  // quaternion_order not in {xyzw, wxyz}.
  EXPECT_THROW(try_construct(options_with(
                   rclcpp::Parameter("imu.quaternion_order", "yxzw"))),
               InvalidParametersException);
  // max_packets_per_frame below the [1, 65536] bound.
  EXPECT_THROW(try_construct(options_with(
                   rclcpp::Parameter("lidar.max_packets_per_frame", 0))),
               InvalidParametersException);

  // Sanity: the valid default options still construct and connect.
  auto s = std::make_unique<FakeAstrallSdk>();
  auto p = std::make_unique<FakeNetworkPreflight>();
  FakeAstrallSdk* sdk = s.get();
  p->next_decision.ready = true;
  p->next_decision.message = "ok";
  auto node = std::make_shared<HypertronDriverNode>(test_node_options(),
                                                    std::move(s), std::move(p));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(wait_until([&] {
    executor.spin_some();
    return sdk->was_called("heartbeat");
  })) << "valid parameters must still construct and connect";
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
