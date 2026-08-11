# Hypertron SW01 ROS2 SSH Bridge

## 项目状态与安全警告

本包把 PC 上的 ROS2 Humble 节点通过 SSH/HTBR 连接到机器人 ARM64 agent；只有 agent 调用 ASTRALL SDK 1.0.7。运动会伤人或损坏设备：首次联调必须架空或使用稳定支撑，保留实体急停和遥控器。软件 `/emergency_stop` 不是物理安全链路的替代品。

本仓库通过了代码级测试与审查，但不构成实机运动安全认证。所有实机步骤均应由具备厂家授权和现场安全责任的操作者执行。

## 架构

```text
PC / x86_64 / ROS2 Humble
  ROS topics/services <-> hypertron_bridge_node <-> libssh + HTBR
                                                       |
robot / ARM64 <-> hypertron_bridge_agent <-> libASTRALL_SDK.so <-> SW01
                                  |                     |
                         IMU/sport/system          UDP 6000/6100/6101
```

HTBR 是 SSH exec channel 上的版本化二进制帧，不是厂家 UDP 3600 协议；字段按网络字节序编码，详见 [HTBR_PROTOCOL.md](HTBR_PROTOCOL.md)。

## 功能范围

- 已实现：速度、模式、软件急停、IMU、运动/系统/电池状态、UDP 6101 里程计与可选 UDP 6000 H.264；机器人端采用 `agent-only` 构建。
- `/joint_commands` 始终安全拒绝且不下行；ASTRALL 1.0.7 没有可用关节 API。`/joint_states` 保持静默，绝不伪造数据。
- 相机默认关闭。里程计未标定前必须保持 `odometry_scale_verified=false`。

## ROS2 接口

| 接口 | 方向 | 类型 | 实际 QoS | 行为 |
|---|---|---|---|---|
| `/cmd_vel` | PC -> agent | `geometry_msgs/msg/Twist` | Reliable, Volatile, KeepLast(1) | 使用 `linear.x`、`linear.y`、`angular.z`，限幅到 `[-1, 1]`；100 ms deadman。 |
| `/robot_mode` | PC -> agent | `std_msgs/msg/String` | Reliable, Volatile, KeepLast(10) | `damping/stand/down/move/auto_charge/exit_charge/recover`（兼容 `recovery`）。 |
| `/emergency_stop` | PC -> agent | `std_srvs/srv/SetBool` | ROS service default: Reliable, Volatile, KeepLast(10) | `true` 锁存零速并请求阻尼；`false` 只解除软件锁存。 |
| `/joint_commands` | PC -> local rejection | `sensor_msgs/msg/JointState` | Reliable, Volatile, KeepLast(1) | 明确拒绝，不向机器人发送。 |
| `/imu/data` | agent -> PC | `sensor_msgs/msg/Imu` | SensorDataQoS: BestEffort, Volatile, KeepLast(5) | 校验并归一化四元数。 |
| `/joint_states` | local silent publisher | `sensor_msgs/msg/JointState` | SensorDataQoS: BestEffort, Volatile, KeepLast(5) | 保留 publisher，当前静默。 |
| `/odom` | agent -> PC | `nav_msgs/msg/Odometry` | SensorDataQoS: BestEffort, Volatile, KeepLast(5) | UDP 6101；比例未确认时状态标记为 false。 |
| `/robot_state` | agent -> PC | `hypertron_ros2_bridge/msg/RobotState` | Reliable, Volatile, KeepLast(10) | 连接、SDK、控制权、错误、状态与安全标记。 |
| `/camera/image_raw` | agent -> PC | `sensor_msgs/msg/Image` | SensorDataQoS: BestEffort, Volatile, KeepLast(5) | 仅在相机启用且硬件支持时发布 BGR8。 |

## ROS2 指令到 ASTRALL 对照

| ROS2 输入 | HTBR 消息 | agent 行为 | ASTRALL 调用 | 期望 `0xBxxx` 状态 |
|---|---|---|---|---|
| `/cmd_vel` | `CMD_VELOCITY` | 安全门控、限幅、latest-value mailbox | `AstrallMove(vx, vy, vyaw)` | `0xB104` |
| `damping` | `CMD_MODE 0xA101` | 串行等待稳态 | `AstrallSportModeControl(0xA101)` | `0xB101` |
| `stand` | `CMD_MODE 0xA102` | 串行等待稳态 | `AstrallSportModeControl(0xA102)` | `0xB102` |
| `down` | `CMD_MODE 0xA103` | 串行等待稳态 | `AstrallSportModeControl(0xA103)` | `0xB103` |
| `move` | `CMD_MODE 0xA104` | 串行等待稳态 | `AstrallSportModeControl(0xA104)` | `0xB104` |
| `auto_charge` | `CMD_MODE 0xA105` | 串行等待稳态 | `AstrallSportModeControl(0xA105)` | `0xB107` 起始 |
| `exit_charge` | `CMD_MODE 0xA106` | 串行等待稳态 | `AstrallSportModeControl(0xA106)` | `0xB10B` 过程 |
| `recover` / `recovery` | `CMD_MODE 0xA1FF` | 串行等待稳态 | `AstrallSportModeControl(0xA1FF)` | `0xB1FF` |
| `/emergency_stop true` | `CMD_ESTOP engage=1` | 最高优先级，锁存零速 | `AstrallMove(0, 0, 0)` 后请求 `0xA101` | `0xB101` |
| `/emergency_stop false` | `CMD_ESTOP engage=0` | 只解除锁存，不重放命令 | 不自动恢复模式或速度 | 由操作者重新确认 |
| `/joint_commands` | `CMD_JOINT` 不下行 | 返回 FeatureUnavailable | 无关节 API | 不适用 |

## 实机前必要条件

以下九项全部完成并记录后，才可扩大运动范围：

1. 厂家或设备管理员确认机器人 SSH 管理 IP、端口、账号、密钥和权限。
2. 在目标 ARM64 系统以对应 `libASTRALL_SDK.so` 编译/链接 agent；绝不复制 x86_64 库。
3. 首次运动架空或使用稳定支撑，实体急停与遥控器始终可达。
4. 每次启动前检查 SDK link、control authority、系统错误、运动状态和急停锁存。
5. 接受关节接口限制：`/joint_commands` 拒绝，`/joint_states` 静默。
6. 以厂家确认或实机抓包标定 UDP 6100/6101 的字节序、packing、坐标系和比例；此前保持 `odometry_scale_verified=false`。
7. 实机确认 UDP 6000 与 H.264 NAL/访问单元边界；此前相机保持默认关闭。
8. 在目标 PC 实测 `ros2 topic list`、服务发现和 DDS 通信；开发 WSL 的跨进程 discovery 不能作为通过证据。
9. 完成低速、明确零速、急停、SSH 断线、agent 重启、遥控器抢权及至少 30 分钟断线/重连耐久测试。

## SOP 0：安全与厂家资料准备

1. 划定无人区，架空或可靠支撑机器人；检查电量、机械状态、实体急停和遥控器。
2. 从厂家取得并核对 ARM64 SDK、`interface.h`、库架构、机器人 SSH 管理信息与物理急停流程；不把 PDF、库、密码、私钥或 token 加入仓库。
3. 记录预期状态：软件启动后不得运动，直到 SOP 5 的全部状态门禁满足。

## SOP 1：网络与 SSH 信任建立

先从厂家标签、串口或可信管理通道离线核对主机指纹，再写入 known_hosts；`ssh-keyscan` 本身不能证明目标身份。

```bash
ssh-keygen -R <ROBOT_IP>
ssh-keyscan -H -p 22 <ROBOT_IP> >> ~/.ssh/known_hosts
ssh-keygen -lf ~/.ssh/known_hosts
ssh -o StrictHostKeyChecking=yes <ROBOT_USER>@<ROBOT_IP> 'true'
```

确认显示的指纹与离线记录一致后，按管理员策略部署公钥。主机密钥变化必须作为安全事件处理，不得自动接受。

## SOP 2：PC 端 ROS2 编译

在 Ubuntu 22.04/x86_64 的 ROS2 Humble PC 上执行。`<HYPERTRON_WS>` 是工作区目录：

```bash
source /opt/ros/humble/setup.bash
mkdir -p <HYPERTRON_WS>/src
git clone <REPOSITORY_URL> <HYPERTRON_WS>/src/hypertron_ros2_bridge
cd <HYPERTRON_WS>
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hypertron_ros2_bridge --symlink-install \
  --cmake-args -DBUILD_ROS2_BRIDGE=ON -DBUILD_AGENT=OFF -DENABLE_CAMERA=OFF
source install/setup.bash
```

不要为了规避依赖检查而跳过 `rosdep`。相机仅在已完成 SOP 6 的相机门禁后，才允许重新构建并启用。

Ubuntu 22.04 的 PC 构建依赖至少包括 `libssh-dev`。只有在明确启用 `ENABLE_CAMERA=ON` 时，还要安装 FFmpeg 开发包：

```bash
sudo apt install libssh-dev libavcodec-dev libavutil-dev libswscale-dev
```

## SOP 3：机器人端 ARM64 agent 部署

机器人不需要 ROS2 或 libssh。`<ASTRALL_SDK_ROOT>` 必须是厂家提供、与目标 ARM64 匹配的 SDK 根目录：

```bash
cmake -S <HYPERTRON_WS>/src/hypertron_ros2_bridge -B /tmp/hypertron-agent-build \
  -DBUILD_ROS2_BRIDGE=OFF -DBUILD_AGENT=ON -DBUILD_TESTING=OFF \
  -DASTRALL_SDK_ROOT=<ASTRALL_SDK_ROOT> -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/hypertron-agent-build -j
scp /tmp/hypertron-agent-build/hypertron_bridge_agent <ROBOT_USER>@<ROBOT_IP>:/tmp/
scp <HYPERTRON_WS>/src/hypertron_ros2_bridge/config/bridge_config.yaml <ROBOT_USER>@<ROBOT_IP>:/tmp/
ssh <ROBOT_USER>@<ROBOT_IP> 'sudo install -d /opt/hypertron/bin /opt/hypertron/config /opt/hypertron/lib'
ssh <ROBOT_USER>@<ROBOT_IP> 'sudo install -m 0755 /tmp/hypertron_bridge_agent /opt/hypertron/bin/'
ssh <ROBOT_USER>@<ROBOT_IP> 'sudo install -m 0644 /tmp/bridge_config.yaml /opt/hypertron/config/'
ssh <ROBOT_USER>@<ROBOT_IP> 'echo /opt/hypertron/lib | sudo tee /etc/ld.so.conf.d/hypertron.conf && sudo ldconfig'
```

由设备管理员把已验证的 ARM64 `libASTRALL_SDK.so` 安装到 `/opt/hypertron/lib/` 后，运行 `ldd /opt/hypertron/bin/hypertron_bridge_agent`；输出不得包含 ROS2 或 libssh。agent 的 stdout 是二进制 HTBR 流，诊断只写 stderr。

## SOP 4：配置文件逐项设置

编辑 `<HYPERTRON_WS>/src/hypertron_ros2_bridge/config/bridge_config.yaml`，设置 `ssh.host: "<ROBOT_IP>"`、`ssh.username: "<ROBOT_USER>"`、密钥路径与已核验的 known_hosts。必须保持：

```yaml
ssh:
  strict_host_key_checking: true
odometry:
  scale_verified: false
camera:
  enabled: false
```

密码应通过受控环境变量或密钥管理提供，不能写入 YAML。核对 `agent_startup_timeout_ms: 65000`、稳态应用心跳超时 500 ms、`heartbeat_hz: 10.0`、`motion_refresh_hz: 50.0` 和 `command_deadman_ms: 100`。

以下行为是代码固定值，不是 YAML 选项，避免产生静默无效配置：

- IMU subscription: fixed at 125 Hz
- sport subscription: fixed at 50 Hz
- automatic motion preparation: disabled
- camera decode queue: fixed capacity 2
- camera output encoding: fixed BGR8

## SOP 5：启动、状态门禁与首次低速运动

启动和检查需要使用独立终端；不要把前台 bridge、持续 `ros2 topic echo` 与后续命令放在同一串命令中。

**终端 A（保持运行）：**

```bash
source /opt/ros/humble/setup.bash
source <HYPERTRON_WS>/install/setup.bash
ros2 run hypertron_ros2_bridge hypertron_bridge_node --ros-args \
  --params-file <HYPERTRON_WS>/src/hypertron_ros2_bridge/config/bridge_config.yaml
```

**终端 B（在终端 A 启动后执行）：**

```bash
source /opt/ros/humble/setup.bash
source <HYPERTRON_WS>/install/setup.bash
ros2 topic list
ros2 service list
ros2 topic echo --once /robot_state
```

运动前，`/robot_state` 必须同时满足：`ssh_connected == true`、`agent_connected == true`、`sdk_linked == true`、`control_authority == true`、`error_code == 0`、`sport_status == 0xB104`，且 `emergency_stop == false`。任一项不满足时不得发送非零 `/cmd_vel`。

在支撑状态下，先站立并观察 `sport_status == 0xB102`，再进入 move 并观察 `sport_status == 0xB104`，两项确认后才可以 20 Hz 发送低速命令；停止时明确发送零速。以下均在终端 B 执行，`--once` 只读取一条状态，必要时重复执行直到达到预期状态：

```bash
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: stand}"
ros2 topic echo --once /robot_state
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: move}"
ros2 topic echo --once /robot_state
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Press Ctrl+C in the velocity publisher terminal before continuing. Then use a separate command block (or another terminal) to send zero, confirm state, and engage software emergency stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic echo --once /robot_state
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
```

发布器停止后 deadman 也会发送零速，但不能代替上述明确零速和现场观察。

## SOP 6：断线与故障恢复

每次故障都先停止运动、保持支撑与实体安全链路，再按以下决策恢复：

1. SSH 丢失：确认零速和阻尼，PC 清空旧速度/模式；核对网络与主机指纹，重连后重新完成 HELLO、模式选择和新速度命令，绝不回放旧命令。
2. 心跳超时：约 500 ms 后 agent 明确调用 `AstrallMove(0, 0, 0)` 并请求 `kAstrallModeDamping`（`0xA101`）；确认 `sport_status == 0xB101`，而不是“锁定站立”。继续失联会清空连接和控制权。排除链路问题后从状态门禁重新开始。
3. 控制权丢失：停止发送运动；由现场确认遥控器/其他控制器已释放后，再申请控制权。agent 不循环抢权。
4. 模式超时：查看 `error_code`、实际 `sport_status` 和相关 ERROR；不要并发重发模式。故障排除后从 `stand` 重新串行执行。
5. 未验证里程计：`odometry_scale_verified=false` 时只作诊断观察，不用于定位、闭环控制或安全判断；抓包/标定后才可更改配置。
6. 相机失败：保持 `camera.enabled: false`，不影响安全控制；验证 UDP 6000 边界、解码与丢包恢复后才可单独启用。
7. 解除急停：执行 `ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"` 后不会自动运动；重新检查全部状态门禁，重新 `stand`、`move`，再发送新的低速命令。

## 测试与 Review

纯核心测试和发布合同：

```bash
cd <HYPERTRON_WS>/src/hypertron_ros2_bridge
cmake -S . -B build-pure -DBUILD_ROS2_BRIDGE=OFF -DBUILD_AGENT=OFF -DBUILD_TESTING=ON
cmake --build build-pure -j
ctest --test-dir build-pure --output-on-failure
python -m pytest test/test_package_contract.py -q
```

[REVIEW.md](REVIEW.md) 记录代码审查与已知硬件门禁；[SW01_MANUAL_NOTES.md](SW01_MANUAL_NOTES.md) 是厂家材料索引，不替代原始厂家资料。

## 已知限制

- 手册第 26 页未定义 UDP 6101 的字节序、packing、坐标系和固定点比例。
- 手册未定义 UDP 6000 数据报与 H.264 NAL/访问单元边界。
- ASTRALL 1.0.7 无关节控制/状态 API，因此 `/joint_commands` 拒绝且 `/joint_states` 静默。
- ROS graph 命令必须在目标 PC 验证；当前 WSL discovery 限制不能当作实机证据。

## 参考资料

- [HTBR 协议](HTBR_PROTOCOL.md)
- [SW01 手册摘要](SW01_MANUAL_NOTES.md)
- [实现与安全 Review](REVIEW.md)
- [ROS 2 Humble QoS settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
- [libssh tutorial](https://api.libssh.org/stable/libssh_tutor_guided_tour.html)
- [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) 与 [DeepRoboticsLab/Lite3_ROS](https://github.com/DeepRoboticsLab/Lite3_ROS)（仅作 ROS2 使用体验参考）

在完成上述门禁前，本项目只能描述为“可进入分阶段台架联调”，不能声称已通过真实 SW01 实机验证。
