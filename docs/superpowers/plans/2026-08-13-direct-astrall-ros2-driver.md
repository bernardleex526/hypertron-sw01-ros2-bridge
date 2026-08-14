# Direct ASTRALL ROS 2 Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the SSH/HTBR runtime with a ROS 2 Humble node that links the vendor x86_64 ASTRALL SDK 1.0.7 directly and controls the SW01 over UDP with non-actuating startup and explicit safety gates.

**Architecture:** A vendor-isolating `AstrallSdkAdapter` implements an injectable `IAstrallSdk`. A thread-owned `DirectDriverRuntime` serializes SDK lifecycle and commands while `HypertronDriverNode` handles ROS interfaces and message conversion. Existing `RobotController`, `RobotState.msg`, and validated mapping helpers are retained where their contracts remain valid.

**Tech Stack:** C++17, ROS 2 Humble, ament_cmake, rclcpp, rosidl, GTest, ASTRALL SDK 1.0.7 x86_64, NetworkManager/iproute2 preflight checks.

---

## File Structure

### Production files

- Modify `CMakeLists.txt`: direct-driver targets, SDK discovery, RPATH, tests, removal of production libssh/agent targets.
- Modify `package.xml`: direct-driver description and dependencies; remove `libssh-dev`.
- Modify `msg/RobotState.msg`: retain wire-compatible fields and document direct-mode legacy fields without changing field order.
- Create `include/hypertron_ros2_bridge/astrall_sdk.hpp`: narrow SDK interface, callback payloads, snapshot, constants, and result helpers.
- Create `include/hypertron_ros2_bridge/direct_driver_runtime.hpp`: runtime configuration, command/result contracts, state callbacks, and lifecycle API.
- Create `include/hypertron_ros2_bridge/network_preflight.hpp`: injectable host-network validation interface.
- Create `include/hypertron_ros2_bridge/driver_node.hpp`: ROS node declaration.
- Modify `include/hypertron_ros2_bridge/robot_controller.hpp`: direct-link naming and mode-pending query needed by runtime.
- Create `src/direct_astrall_sdk.cpp`: only production file including vendor `interface.h`.
- Create `src/direct_driver_runtime.cpp`: worker loop, reconnect, heartbeat, telemetry poll, command queue, deadman, mode confirmation, stop behavior.
- Create `src/network_preflight.cpp`: read-only Linux interface/address/route checks.
- Create `src/driver_node.cpp`: ROS subscriptions, services, publishers, parameters, and SDK-to-ROS conversion.
- Create `src/driver_main.cpp`: rclcpp init/spin/shutdown only.
- Modify `src/robot_controller.cpp`: direct readiness semantics and query methods.
- Modify `src/data_mapping.cpp`: map SDK IMU odometry fields without claiming UDP 6101 validation.
- Replace `config/bridge_config.yaml` with `config/driver_config.yaml`: only effective direct-driver parameters.
- Create `launch/driver.launch.py`: locate package config and launch `hypertron_driver_node`.
- Rewrite `README.md`: build, network, read-only commissioning, authority and motion SOP.
- Update `SW01_MANUAL_NOTES.md` and `REVIEW.md`: cite the supplied official documents and direct-UDP limitations.

### Test files

- Create `test/fake_astrall_sdk.hpp`: deterministic fake with call log, programmable results, callbacks, and snapshots.
- Create `test/fake_network_preflight.hpp`: programmable route/address validation.
- Create `test/test_direct_driver_runtime.cpp`: lifecycle, reconnect, authority, safety, deadman, mode, emergency stop, shutdown.
- Create `test/test_network_preflight.cpp`: parser/decision tests using injected command output or fact model.
- Modify `test/test_robot_controller.cpp`: direct readiness and pending-mode tests.
- Modify `test/test_data_receiver.cpp`: IMU validation, quaternion order, diagnostic odometry, no-TF default contract.
- Replace `test/test_package_contract.py`: direct SDK discovery and no production SSH/HTBR requirements.

### Retired production files

The following stay in git history but are removed from production targets and then deleted after replacement tests pass:

- `src/bridge_node.cpp`, `src/main.cpp`, `src/ssh_tunnel.cpp`
- `src/agent_main.cpp`, `src/agent_runtime.cpp`, `src/posix_byte_stream.cpp`
- `include/hypertron_ros2_bridge/bridge_node.hpp`
- `include/hypertron_ros2_bridge/ssh_tunnel.hpp`
- HTBR-only runtime declarations in `astrall_sdk_adapter.hpp`

Protocol encoder files may remain only if `RobotController` still depends on their small value types. A later focused cleanup can split those types; this implementation must not broaden into unrelated protocol refactoring.

## Task 1: Establish Direct SDK Build Contract

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `package.xml`
- Replace: `test/test_package_contract.py`

- [ ] **Step 1: Write failing package-contract tests**

Assert that the package:

```python
def test_direct_driver_contract():
    cmake = read("CMakeLists.txt")
    package = read("package.xml")
    assert "ASTRALL_SDK_ROOT" in cmake
    assert "hypertron_driver_node" in cmake
    assert "direct_astrall_sdk.cpp" in cmake
    assert "libssh-dev" not in package
    assert "hypertron_bridge_agent" not in cmake
    assert "PkgConfig::LIBSSH" not in cmake
```

Add a test that invokes CMake with an invalid SDK root and expects a diagnostic containing `ASTRALL SDK x86_64 header/library not found`.

- [ ] **Step 2: Run the contract test and verify RED**

Run:

```bash
python3 -m pytest test/test_package_contract.py -q
```

Expected: failures for missing direct target and remaining SSH dependency.

- [ ] **Step 3: Implement minimal SDK discovery and target skeleton**

Use a cache path, never a hard-coded home path:

```cmake
set(ASTRALL_SDK_ROOT "" CACHE PATH "ASTRALL SDK C++ root")
find_path(ASTRALL_INCLUDE_DIR interface.h
  PATHS "${ASTRALL_SDK_ROOT}/include" NO_DEFAULT_PATH)
find_library(ASTRALL_LIBRARY NAMES ASTRALL_SDK
  PATHS "${ASTRALL_SDK_ROOT}/lib/linux/x86_64" NO_DEFAULT_PATH)
if(NOT ASTRALL_INCLUDE_DIR OR NOT ASTRALL_LIBRARY)
  message(FATAL_ERROR "ASTRALL SDK x86_64 header/library not found")
endif()
```

Define `hypertron_driver_node`, remove production libssh and agent targets, and set build/install RPATH to the discovered library directory. Remove `libssh-dev` from `package.xml` and update its description.

- [ ] **Step 4: Verify contract GREEN and CMake configuration**

Run:

```bash
python3 -m pytest test/test_package_contract.py -q
cmake -S . -B /tmp/hypertron-direct-config \
  -DASTRALL_SDK_ROOT=/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++ \
  -DBUILD_TESTING=ON
```

Expected: pytest passes and CMake reports the x86_64 SDK library path.

## Task 2: Define Injectable SDK And Network Boundaries

**Files:**
- Create: `include/hypertron_ros2_bridge/astrall_sdk.hpp`
- Create: `include/hypertron_ros2_bridge/network_preflight.hpp`
- Create: `src/direct_astrall_sdk.cpp`
- Create: `src/network_preflight.cpp`
- Create: `test/fake_astrall_sdk.hpp`
- Create: `test/fake_network_preflight.hpp`
- Create: `test/test_network_preflight.cpp`

- [ ] **Step 1: Write failing boundary tests**

Test a fact-based preflight decision:

```cpp
TEST(NetworkPreflight, RequiresExactHostAddressAndWiredRoute) {
  NetworkFacts facts{{"10.18.0.200/24"}, "eno1", "10.18.0.200"};
  EXPECT_TRUE(evaluate_network(facts, {"eno1", "10.18.0.200/24",
                                       "10.18.0.100"}).ready);
  facts.route_interface = "Meta";
  EXPECT_FALSE(evaluate_network(facts, {"eno1", "10.18.0.200/24",
                                        "10.18.0.100"}).ready);
}
```

Compile-time fake tests must demonstrate programmable init/heartbeat/auth/mode/move results, captured calls, callback injection, and snapshots.

- [ ] **Step 2: Run focused tests and verify RED**

Run:

```bash
cmake --build /tmp/hypertron-direct-config --target test_network_preflight
ctest --test-dir /tmp/hypertron-direct-config -R network_preflight --output-on-failure
```

Expected: target or symbols are missing.

- [ ] **Step 3: Implement narrow interfaces**

Define `IAstrallSdk` with exactly:

```cpp
enum class SubscriptionFrequency : std::uint16_t {
  Disabled = 0,
  Hz1,
  Hz25,
  Hz50,
  Hz125,
  Hz250,
};

virtual Result init(const SdkCallbacks&, std::uint32_t timeout_ms) = 0;
virtual void deinit() noexcept = 0;
virtual Result heartbeat(std::uint32_t timeout_ms) = 0;
virtual Result request_authority(bool sdk, std::uint32_t timeout_ms) = 0;
virtual Result subscribe_imu(SubscriptionFrequency, ImuCallback,
                             std::uint32_t timeout_ms) = 0;
virtual Result subscribe_sport(SubscriptionFrequency, SportCallback,
                               std::uint32_t timeout_ms) = 0;
virtual Result move(Velocity, std::uint32_t timeout_ms) = 0;
virtual Result set_mode(std::uint16_t, std::uint32_t timeout_ms) = 0;
virtual SdkSnapshot snapshot() = 0;
```

`DirectAstrallSdk` maps only these calls to SDK 1.0.7, copies callback buffers, and never requests authority during `init`. `LinuxNetworkPreflight` gathers address and route facts without changing system configuration.

- [ ] **Step 4: Run focused tests and adapter compile check**

Run the network test and build `hypertron_direct_core`. Expected: all pass/build with no robot connected and no SDK initialization executed by tests.

## Task 3: Adapt Safety Controller For Direct Runtime

**Files:**
- Modify: `include/hypertron_ros2_bridge/robot_controller.hpp`
- Modify: `src/robot_controller.cpp`
- Modify: `test/test_robot_controller.cpp`

- [ ] **Step 1: Add failing direct-runtime safety tests**

Add tests proving:

```cpp
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
```

- [ ] **Step 2: Run and verify RED**

Run `ctest --test-dir /tmp/hypertron-direct-config -R robot_controller --output-on-failure`.

Expected: missing direct readiness/query APIs.

- [ ] **Step 3: Implement minimal controller changes**

Rename negotiation-specific semantics to driver readiness, expose a thread-safe pending-mode query, and provide one connection invalidation method that clears state, pending mode, and velocity without clearing the software emergency-stop latch.

- [ ] **Step 4: Run and verify GREEN**

Run the focused controller test. Expected: all existing and new cases pass.

## Task 4: Implement Direct Driver Runtime With Fake SDK

**Files:**
- Create: `include/hypertron_ros2_bridge/direct_driver_runtime.hpp`
- Create: `src/direct_driver_runtime.cpp`
- Create: `test/test_direct_driver_runtime.cpp`
- Use: `test/fake_astrall_sdk.hpp`
- Use: `test/fake_network_preflight.hpp`

- [ ] **Step 1: Write lifecycle and non-actuating startup tests**

Assert that `start()` with ready network calls `init`, subscribes IMU/sport, and starts heartbeat, but call history contains no `request_authority`, `set_mode`, or `move`.

- [ ] **Step 2: Run lifecycle tests and verify RED**

Expected: runtime target/classes missing.

- [ ] **Step 3: Implement worker lifecycle and reconnect**

Implement one `std::jthread` or stop-token equivalent with:

```text
preflight -> init -> subscribe -> connected loop
failure -> invalidate controller -> deinit -> bounded backoff -> preflight
```

All sleep/wait behavior must be interruptible. Reconnection resets authority and never replays commands.

- [ ] **Step 4: Add authority and mode tests**

Test `request_authority(true/false)`, rejection while disconnected, serialized mode requests, stable-status confirmation, and timeout. Verify `false` maps to vendor joystick authority.

- [ ] **Step 5: Implement authority and serialized modes**

Use a bounded command queue for low-rate requests and return futures/results to ROS services. Map mode commands to documented stable statuses, including damping `B101`, stand `B102`, down `B103`, move `B104`, and recovery `B1FF`.

- [ ] **Step 6: Add motion, deadman, emergency-stop, and shutdown tests**

Use a manual clock. Prove 20 ms refresh, latest-value replacement, explicit zero after 100 ms, immediate stop on link/authority/error loss, emergency-stop priority/non-replay, and conditional shutdown zero+damping only when authority is owned.

- [ ] **Step 7: Implement motion and emergency behavior**

The worker is the only thread that calls motion/mode SDK methods. Emergency-stop latches in the controller before enqueueing stop work. No queue may retain a stale nonzero velocity after invalidation.

- [ ] **Step 8: Run full runtime tests**

Run:

```bash
ctest --test-dir /tmp/hypertron-direct-config \
  -R 'direct_driver_runtime|robot_controller' --output-on-failure
```

Expected: all cases pass without binding UDP 3600.

## Task 5: Implement ROS Node And Telemetry Mapping

**Files:**
- Create: `include/hypertron_ros2_bridge/driver_node.hpp`
- Create: `src/driver_node.cpp`
- Create: `src/driver_main.cpp`
- Modify: `src/data_mapping.cpp`
- Modify: `test/test_data_receiver.cpp`
- Modify: `msg/RobotState.msg`

- [ ] **Step 1: Add failing mapping and ROS contract tests**

Cover finite checks, XYZW/WXYZ reorder, quaternion normalization, configured covariances, `odomX/odomY` diagnostic mapping, `odometry_scale_verified=false`, and disabled TF default. Assert RobotState direct-mode fields:

```cpp
EXPECT_FALSE(state.ssh_connected);
EXPECT_FALSE(state.agent_connected);
EXPECT_TRUE(state.sdk_linked);
EXPECT_FALSE(state.joint_interface_available);
EXPECT_FALSE(state.camera_available);
```

- [ ] **Step 2: Run focused mapping tests and verify RED**

Expected: missing SDK IMU odometry mapping or direct node contract.

- [ ] **Step 3: Implement ROS node interfaces**

Create publishers/subscriptions/services with the approved names and QoS. Node construction starts the runtime but does not block on SDK initialization. `/control_authority` and `/emergency_stop` return truthful SDK/runtime results; topic callbacks log throttled rejection reasons.

- [ ] **Step 4: Implement telemetry publishing**

Publish SDK callback data only after copying and validation. Poll state at 2 Hz. Publish `/odom` diagnostically and create a TF broadcaster only when `odom.publish_tf=true` and `odometry.scale_verified=true`; reject invalid configuration where TF is requested while scale verification is false.

- [ ] **Step 5: Run mapping and node component tests**

Expected: message mapping tests pass and node can be instantiated against fakes without network or SDK side effects.

## Task 6: Configuration, Launch, Documentation, And Retirement

**Files:**
- Create: `config/driver_config.yaml`
- Create: `launch/driver.launch.py`
- Modify: `README.md`
- Modify: `SW01_MANUAL_NOTES.md`
- Modify: `REVIEW.md`
- Delete after tests pass: obsolete SSH/HTBR/agent production sources and headers listed above
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write failing installation contract tests**

Assert that installed resources include `driver_config.yaml` and `driver.launch.py`, defaults are non-actuating, heartbeat is 100 ms, deadman is 100 ms, odometry verification/TF are false, and no SSH key/host/agent parameters remain.

- [ ] **Step 2: Run contract tests and verify RED**

Expected: new resources absent and old SSH config present.

- [ ] **Step 3: Add launch/config and rewrite operator documentation**

Document exact build command:

```bash
colcon build --packages-select hypertron_ros2_bridge --symlink-install \
  --cmake-args \
  -DASTRALL_SDK_ROOT=/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++
```

Document read-only launch, state checks, manual authority service, authority return, supported modes, explicit zero, emergency stop, and physical safety prerequisites. Do not include a one-command autonomous motion sequence.

- [ ] **Step 4: Remove obsolete production runtime**

Delete SSH/HTBR/agent production files only after no production target references them. Retain any small protocol value-type file still required by `RobotController`; remove libssh and FFmpeg discovery from the default build.

- [ ] **Step 5: Run package contracts and source-tree scan**

Run:

```bash
python3 -m pytest test/test_package_contract.py -q
git diff --check
```

Expected: pass, with no production references to SSH tunnel, remote agent, or HTBR configuration.

## Task 7: Workspace Integration And Host-Side Verification

**Files:**
- Create symlink: `/home/lee/ros2_ws/src/hypertron_ros2_bridge` -> `/home/lee/hypertron-sw01-ros2-bridge`
- Create: `/home/lee/ros2_ws/src/sw01_ros2_driver/COLCON_IGNORE` only after replacement verification

- [ ] **Step 1: Link the repository into the ROS workspace**

Verify the target and parent first, then create the symlink. Do not move the git repository.

- [ ] **Step 2: Build the focused package**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hypertron_ros2_bridge --symlink-install \
  --cmake-args \
  -DASTRALL_SDK_ROOT=/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++ \
  -DBUILD_TESTING=ON
```

Expected: one package builds successfully and links the x86_64 vendor library.

- [ ] **Step 3: Run focused package tests**

Run:

```bash
colcon test --packages-select hypertron_ros2_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failed tests.

- [ ] **Step 4: Run a no-robot launch smoke test**

With `eno1` disconnected, launch for a bounded interval:

```bash
source install/setup.bash
timeout 10s ros2 launch hypertron_ros2_bridge driver.launch.py
```

Expected: node remains alive, reports failed preflight/retry, creates no control authority request, and exits cleanly on timeout.

- [ ] **Step 5: Inspect ROS graph during fake/no-network test mode**

Confirm the expected publishers, subscriptions, and services exist, including `/control_authority`, while no robot command is sent.

- [ ] **Step 6: Exclude the unmanaged duplicate package**

After all replacement checks pass, add `COLCON_IGNORE` to
`/home/lee/ros2_ws/src/sw01_ros2_driver` so only one process can own UDP 3600 and the SW01 ROS interfaces.

- [ ] **Step 7: Report hardware verification as pending**

Do not run SDK initialization or any authority/motion command until the robot is
connected, supported, the physical emergency stop is reachable, and an operator
with the remote controller is present. Report host-side evidence separately from
pending real-robot evidence.

## Final Verification Gate

- [ ] `git diff --check` passes.
- [ ] Focused pytest, CTest, colcon build, and colcon test all pass.
- [ ] `ldd install/hypertron_ros2_bridge/lib/hypertron_ros2_bridge/hypertron_driver_node` resolves the x86_64 ASTRALL library.
- [ ] `ros2 pkg executables hypertron_ros2_bridge` lists only the direct driver production executable.
- [ ] No-network launch remains alive and retries without authority/mode/motion SDK calls.
- [ ] Documentation distinguishes software emergency stop from physical STO.
- [ ] Real-robot connectivity, telemetry, authority, and motion remain explicitly unverified until staged commissioning.

No git commit, push, or pull request is part of this plan unless separately requested.
