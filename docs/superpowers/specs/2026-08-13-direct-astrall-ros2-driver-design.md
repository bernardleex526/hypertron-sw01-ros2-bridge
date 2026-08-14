# Direct ASTRALL ROS 2 Driver Design

## Goal

Replace the SSH/HTBR/robot-agent runtime with one ROS 2 Humble node that links
the vendor x86_64 ASTRALL SDK 1.0.7 and communicates directly with the SW01 over
UDP. Preserve the existing ROS interface where it represents real vendor
capabilities, and make startup non-actuating by default.

The vendor contract is fixed in SDK 1.0.7:

- robot: `10.18.0.100`
- host: `10.18.0.200`
- SDK UDP port: `3600`
- supported host library:
  `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++/lib/linux/x86_64/libASTRALL_SDK.so`

Camera H.264, lidar point clouds, and the standalone UDP 6101 odometry stream
are outside this first release.

## Architecture

The runtime data path is:

```text
ROS 2 applications
  <-> HypertronDriverNode
  <-> DirectDriverRuntime
  <-> AstrallSdkAdapter
  <-> x86_64 libASTRALL_SDK.so
  <-> UDP 10.18.0.100:3600
  <-> SW01
```

`AstrallSdkAdapter` is the only production module that includes the vendor
`interface.h`. It copies callback-owned data before returning from a vendor
callback. Its interface supports initialization, deinitialization, heartbeat,
subscriptions, status snapshots, authority transfer, mode commands, and
velocity commands. Tests replace it with `FakeAstrallSdk`.

`DirectDriverRuntime` owns a worker thread for blocking SDK operations. It
serializes initialization, reconnection, authority requests, mode transitions,
and shutdown. ROS executor callbacks enqueue bounded commands and never block on
the SDK initialization timeout. Connection failure keeps the node alive and
causes bounded exponential reconnect attempts.

`HypertronDriverNode` owns ROS parameters, publishers, subscriptions, services,
QoS, message conversion, and command rejection reporting. The package remains
named `hypertron_ros2_bridge` so existing `RobotState` type references continue
to work. The executable becomes `hypertron_driver_node` and the node name is
`hypertron_driver`.

The obsolete SSH tunnel, HTBR protocol, and robot-side agent are not part of the
production build. Their history remains available in git. Pure safety logic that
does not depend on HTBR, especially `RobotController`, is retained and adapted.

## Startup And Connection

Startup must not actuate the robot. The driver initializes the SDK and subscribes
to telemetry, but it does not request control authority, switch modes, or send a
velocity command.

Before each initialization attempt, the runtime checks that the host has
`10.18.0.200/24` and that the route to `10.18.0.100` uses the configured wired
interface. A failed preflight is reported through `/robot_state` and logs, then
retried. It does not terminate the ROS node.

The SDK heartbeat period is 100 ms, matching the vendor recommendation of 10 Hz.
Heartbeat failure or a vendor link callback reporting disconnect clears pending
motion and mode work. Reconnection never restores authority, mode, or velocity.

## ROS Interface

Existing interfaces are retained:

| Name | Type | Direction | Behavior |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | subscribe | Uses `linear.x`, `linear.y`, and `angular.z` after safety checks and limiting to `[-1, 1]`. |
| `/robot_mode` | `std_msgs/msg/String` | subscribe | Accepts `damping`, `stand`, `down`, `move`, `auto_charge`, `exit_charge`, `recover`, and `recovery`. |
| `/emergency_stop` | `std_srvs/srv/SetBool` | service | `true` latches software stop, sends zero, and requests damping. `false` only clears the software latch. |
| `/joint_commands` | `sensor_msgs/msg/JointState` | subscribe | Always rejected because SDK 1.0.7 has no joint command API. |
| `/imu/data` | `sensor_msgs/msg/Imu` | publish | Vendor IMU data with validation and configured covariance. |
| `/odom` | `nav_msgs/msg/Odometry` | publish | Diagnostic `odomX/odomY` mapping; unverified by default. |
| `/robot_state` | `hypertron_ros2_bridge/msg/RobotState` | publish | Strongly typed connection, authority, safety, system, battery, mode, and wheel-speed state. |

One service is added:

| Name | Type | Behavior |
|---|---|---|
| `/control_authority` | `std_srvs/srv/SetBool` | `true` requests `ASTRALL_AUTH_SDK`; `false` calls `ASTRALL_AUTH_JOYSTICK`, which means transfer to the remote controller because SDK 1.0.7 has no separate release operation. |

For message compatibility, `RobotState.ssh_connected` and
`RobotState.agent_connected` remain present and are always false. `sdk_linked`
is the actual vendor UDP link state. `joint_interface_available` and
`camera_available` are false in this release. `last_error` contains the latest
preflight, SDK, heartbeat, subscription, or rejected-command detail.

No `/joint_states` publisher is created because the SDK supplies no joint state.

## Safety State Machine

Velocity is accepted only when all conditions hold:

- SDK is linked.
- SDK control authority is held.
- Software emergency stop is not latched.
- Vendor system error code is zero.
- Vendor sport status is `ASTRALL_SPORT_STATUS_MOVE` (`0xB104`).
- No mode transition is pending.

Accepted velocity is clamped to the documented `[-1, 1]` range. The runtime
stores only the newest command and refreshes it every 20 ms while it remains
valid. If no new ROS velocity arrives for 100 ms, the runtime sends an explicit
zero and clears the command. This is independent of the vendor's 50 ms automatic
stop behavior.

Mode transitions are serialized. A new mode request is rejected while another
transition is pending. Completion is confirmed by polling the vendor sport
status until the documented stable status is reached or the configured timeout
expires.

Engaging software emergency stop has priority over queued work. It latches
locally before any SDK call, clears pending velocity and mode work, sends zero,
then requests damping. Clearing the latch never reacquires authority, changes
mode, or replays a previous command. The physical STO and remote controller
remain the real safety mechanisms.

During normal shutdown, the runtime attempts zero velocity and damping only if
the SDK is linked and this process currently owns control authority. It then
deinitializes the SDK. It does not unconditionally alter a robot controlled by
another device.

## Telemetry Mapping

Vendor callbacks are copied into bounded latest-value storage before conversion
to ROS messages.

IMU messages use SensorDataQoS. All floating-point fields are checked for finite
values. The quaternion is reordered according to `imu.quaternion_order`, which
defaults to `xyzw`, normalized, and rejected if its norm is invalid. Covariance
diagonals are configurable.

The vendor documentation does not establish the scale and frame semantics of
`AstrallImuData.odomX/odomY`. The driver publishes them for diagnostics with
`RobotState.odometry_scale_verified=false`. `odom.publish_tf` defaults to false;
the data must not feed Nav2 or closed-loop control until measured against an
external reference.

The SDK sport callback supplies four wheel speeds, which populate
`RobotState.wheel_speed`. A 2 Hz status poll supplies system status, error and
warning codes, sport status, battery percentage, temperature, voltage, cycle
count, and charge status.

## Parameters

The first release exposes only parameters with working behavior:

- topic and frame names
- wired interface name, default `eno1`
- expected host address, default `10.18.0.200/24`
- expected robot address, fixed-value validation for `10.18.0.100`
- SDK root path used at build time
- initialization and SDK call timeouts
- reconnect initial and maximum delay
- heartbeat period, default 100 ms
- motion refresh period, default 20 ms
- command deadman, default 100 ms
- mode transition timeout
- IMU and sport subscription frequencies
- quaternion order and covariance diagonals
- odometry scale-verified flag and TF publication flag, both false by default

The SDK target IP and port are not presented as effective runtime options because
the vendor x86_64 binary hard-codes them.

## Build And Deployment

CMake accepts `ASTRALL_SDK_ROOT` instead of embedding a developer-specific path.
Configuration fails with a clear message when the x86_64 header or library is
missing or has the wrong architecture. The node links to the vendor library and
installs an environment hook or RPATH sufficient for `ros2 run` and launch use.
The proprietary vendor library is referenced from its supplied SDK directory and
is not copied into git.

The repository is exposed in `/home/lee/ros2_ws/src` by symlink and built as a
colcon package. The old unmanaged `sw01_ros2_driver` package is excluded from the
workspace after the replacement passes tests, preventing two nodes from binding
UDP port 3600 or publishing the same interfaces.

## Verification

Unit tests use a fake SDK and require no robot. They cover:

- startup connects and publishes but does not request authority
- failed preflight and SDK initialization retry without terminating the node
- authority service request and transfer semantics
- every velocity safety gate and input clamp
- latest-value motion refresh and 100 ms deadman zero
- serialized mode transitions and timeout handling
- emergency-stop priority, latch, and non-replay behavior
- disconnect clearing commands and reconnection requiring fresh authority
- conditional shutdown behavior
- IMU finite-value checks, quaternion ordering and normalization
- unverified odometry and disabled TF defaults
- RobotState field mapping and unavailable joint behavior

Package contract tests check SDK discovery, x86_64 architecture, message and
launch installation, and absence of production SSH/HTBR dependencies. Focused
colcon build and test commands provide host-side evidence.

Hardware verification is staged and must use physical support, an accessible
physical emergency stop, and an operator with the remote controller:

1. Confirm `10.18.0.100` routes through `eno1` with source `10.18.0.200`.
2. Start in read-only mode and verify link, device, system, battery, IMU, and
   wheel-speed telemetry.
3. Request and return authority without changing mode.
4. With the robot supported, verify stand and move transitions serially.
5. Send low velocity, explicit zero, ROS deadman, software stop, physical stop,
   disconnect, reconnect, and remote-controller authority takeover.

No successful build or unit test is treated as proof of physical motion safety.
