# Hypertron SW01 ROS2 SSH Bridge

本包把 Hypertron SW01 的 ASTRALL SDK 1.0.7 封装成类似常见机器狗 ROS2 SDK 的使用体验：开发 PC 运行 ROS2 Humble 节点，通过一个带自动重连和应用心跳的 SSH exec 通道启动机器人端 agent；ROS 命令向下发送，IMU、运动、电池、系统、里程计及可选相机数据向上发布。

> 运动机器人有伤人和损坏设备风险。首次验证必须架空或使用安全支撑，保留实体急停和遥控器，不得仅依赖软件 `/emergency_stop`。

## 架构

```text
PC / x86_64 / ROS2 Humble
  /cmd_vel, /robot_mode, /joint_commands, /emergency_stop
            │
            ▼
  hypertron_bridge_node ── libssh + HTBR framed stream ──┐
            ▲                                             │
            │                                             ▼
  /imu/data, /odom, /robot_state, /camera/image_raw  robot / ARM64
                                                   hypertron_bridge_agent
                                                      │       │
                                      libASTRALL_SDK.so│       ├─ UDP 6000 H.264
                                                      │       ├─ UDP 6100 point cloud
                                                      ▼       └─ UDP 6101 odometry
                                                 Hypertron SW01
```

SSH 在这里是加密的长连接和进程通道，不把厂家 UDP 3600 暴露给 PC。只有机器人端 `AstrallSdkAdapter` 调用厂家 SDK。跨 x86_64/ARM64 的数据逐字段用网络字节序序列化，绝不发送裸 C++ struct。

## ROS 接口

| 名称 | 类型 | 行为 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 使用 `linear.x`、`linear.y`、`angular.z`，限制到 `[-1,1]` |
| `/joint_commands` | `sensor_msgs/msg/JointState` | 保留接口；当前安全拒绝，不向机器人发送 |
| `/robot_mode` | `std_msgs/msg/String` | `damping/stand/down/move/auto_charge/exit_charge/recover`（兼容 `recovery`） |
| `/emergency_stop` | `std_srvs/srv/SetBool` | `true` 锁存零速并请求阻尼，`false` 只解除软件锁存 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 默认 SensorDataQoS，四元数验证和归一化 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 保留 publisher；厂家接口到位前保持静默 |
| `/odom` | `nav_msgs/msg/Odometry` | 来自 UDP 6101；比例未标定时状态明确为 false |
| `/robot_state` | `hypertron_ros2_bridge/msg/RobotState` | SSH、agent、SDK、控制权、电池、错误、模式等 |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 构建启用 FFmpeg 且硬件支持时发布 BGR8 |

## 安全和断线行为

- agent 以 10 Hz 调用 `AstrallHeartbeat`，以 50 Hz 调用 `AstrallMove`；PC `/cmd_vel` 默认 100 ms deadman。
- 只有 SDK 已连接、持有控制权、运动状态为 `0xB104`、系统错误为零且急停未锁存时，非零速度才会下发。
- PC 应用 PING/PONG 超过 500 ms 失联，agent 立即发送零速并请求 `0xA101` 阻尼。
- SSH channel 建立后先允许 65 秒 SDK 初始化期；收到首个 PONG 后才切换到 500 ms 超时，避免 60 秒 `AstrallSdkInit` 尚未完成就误重连。
- 急停帧优先于普通命令；速度突发只保留最新值。模式命令串行执行，达到手册规定的 `0xBxxx` 状态后才确认。
- SSH 断线会把本机连接/SDK/控制权状态清零，取消等待中的服务请求并丢弃旧速度和旧模式；重连后不回放命令，必须重新选择模式并重新发送速度。
- 遥控器抢权时 agent 不循环申请控制权。
- `/emergency_stop false` 不恢复旧模式或旧速度；操作者必须重新 `stand`、`move` 并发送新命令。

## 配置

完整示例在 [`config/bridge_config.yaml`](config/bridge_config.yaml)。至少修改：

```yaml
hypertron_bridge:
  ros__parameters:
    ssh:
      host: "192.168.1.100"
      port: 22
      username: "robot"
      password: ""
      private_key: "~/.ssh/id_ed25519"
      known_hosts: "~/.ssh/known_hosts"
      strict_host_key_checking: true
      remote_command: "/opt/hypertron/bin/hypertron_bridge_agent --stdio --config /opt/hypertron/config/bridge_config.yaml"
```

不建议把密码写进 YAML。环境变量 `HYPERTRON_SSH_PASSWORD` 会覆盖 YAML 密码，日志不会输出密码。生产环境应使用密钥并保持 `strict_host_key_checking: true`。

桥接帧的头、消息号和 payload 精确定义见 [`HTBR_PROTOCOL.md`](HTBR_PROTOCOL.md)。

### known_hosts 初始化

先通过厂家标签、串口或可信管理渠道核对机器狗 SSH 主机密钥指纹，再登记：

```bash
ssh-keygen -R 192.168.1.100
ssh-keyscan -H -p 22 192.168.1.100 >> ~/.ssh/known_hosts
ssh-keygen -lf ~/.ssh/known_hosts
ssh-copy-id -i ~/.ssh/id_ed25519.pub robot@192.168.1.100
```

`ssh-keyscan` 本身不能证明密钥属于目标设备，指纹必须离线核对。主机密钥变化会被当作永久安全错误，不会自动接受。

## 本机：ROS2 Humble 编译

目标系统为 Ubuntu 22.04 / x86_64。安装 ROS2 Humble 后：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-dev-tools \
  libssh-dev pkg-config \
  libavcodec-dev libavutil-dev libswscale-dev

source /opt/ros/humble/setup.bash
mkdir -p ~/hypertron_ws/src
cp -a hypertron_ros2_bridge ~/hypertron_ws/src/
cd ~/hypertron_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select hypertron_ros2_bridge --symlink-install \
  --cmake-args -DBUILD_ROS2_BRIDGE=ON -DBUILD_AGENT=OFF -DENABLE_CAMERA=OFF
source install/setup.bash
```

相机需要时把 `ENABLE_CAMERA` 改为 `ON`，并在 YAML 中设置 `camera.enabled: true`。关闭相机时 `/camera/image_raw` 仍可发现，但不会发布图像。

## 机器狗端：agent-only ARM64 编译

机器人端不需要 ROS2 或 libssh。推荐直接在 ARM64 机器人或同发行版 ARM64 构建机上原生编译。把厂家 SDK 整理为：

```text
/opt/hypertron/vendor/astrall_sdk/
├── include/interface.h
└── lib/linux/ARM64/libASTRALL_SDK.so
```

然后：

```bash
sudo apt update
sudo apt install -y build-essential cmake
cmake -S /path/to/hypertron_ros2_bridge -B /tmp/hypertron-agent-build \
  -DBUILD_ROS2_BRIDGE=OFF \
  -DBUILD_AGENT=ON \
  -DBUILD_TESTING=OFF \
  -DASTRALL_SDK_ROOT=/opt/hypertron/vendor/astrall_sdk \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/hypertron-agent-build -j
```

这就是 `agent-only` 构建；`ldd /tmp/hypertron-agent-build/hypertron_bridge_agent` 不应出现 ROS2 或 libssh。

交叉编译时提供 ARM64 toolchain/sysroot，并确保链接的是 `C++/lib/linux/ARM64/libASTRALL_SDK.so`，不能把开发 PC 的 x86_64 库复制到机器人。示例：

```bash
cmake -S hypertron_ros2_bridge -B build-agent-arm64 \
  -DCMAKE_TOOLCHAIN_FILE=/path/to/aarch64-toolchain.cmake \
  -DBUILD_ROS2_BRIDGE=OFF -DBUILD_AGENT=ON -DBUILD_TESTING=OFF \
  -DASTRALL_SDK_ROOT=/path/to/ASTRALL_SDK_1.0.7
cmake --build build-agent-arm64 -j
```

## 部署到机器狗

```bash
ssh robot@192.168.1.100 'mkdir -p /tmp/hypertron-deploy'
scp /tmp/hypertron-agent-build/hypertron_bridge_agent \
  robot@192.168.1.100:/tmp/hypertron-deploy/
scp hypertron_ros2_bridge/config/bridge_config.yaml \
  robot@192.168.1.100:/tmp/hypertron-deploy/

ssh robot@192.168.1.100
sudo install -d /opt/hypertron/bin /opt/hypertron/config /opt/hypertron/lib
sudo install -m 0755 /tmp/hypertron-deploy/hypertron_bridge_agent \
  /opt/hypertron/bin/
sudo install -m 0644 /tmp/hypertron-deploy/bridge_config.yaml \
  /opt/hypertron/config/
sudo install -m 0644 /opt/hypertron/vendor/astrall_sdk/lib/linux/ARM64/libASTRALL_SDK.so \
  /opt/hypertron/lib/
echo /opt/hypertron/lib | sudo tee /etc/ld.so.conf.d/hypertron.conf
sudo ldconfig
/opt/hypertron/bin/hypertron_bridge_agent --help
```

不要把 agent 配置成 PTY 程序；stdout 是严格二进制 HTBR 流，诊断日志只写 stderr。通常无需单独 systemd 服务，因为 PC 的 libssh exec channel 会启动并监护 agent。

## 启动与验证

在 PC：

```bash
source /opt/ros/humble/setup.bash
source ~/hypertron_ws/install/setup.bash
export HYPERTRON_SSH_PASSWORD=''  # 使用密钥时保持为空或不设置
ros2 run hypertron_ros2_bridge hypertron_bridge_node --ros-args \
  --params-file ~/hypertron_ws/src/hypertron_ros2_bridge/config/bridge_config.yaml
```

检查 ROS 图和连接状态：

```bash
ros2 topic list
ros2 topic echo /robot_state
ros2 topic hz /imu/data
ros2 topic echo /odom
```

首次运动必须先确认 `/robot_state` 中 `ssh_connected`、`agent_connected`、`sdk_linked`、`control_authority` 为 true，`error_code` 为 0，并保证现场安全。然后逐步执行：

```bash
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: stand}"
# 等待 robot_state.sport_status == 0xB102
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: move}"
# 等待 robot_state.sport_status == 0xB104
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止发布器后 deadman 会发送零速；也可明确发布一次零速。软件急停：

```bash
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"
ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: false}"
```

解除后不会自动恢复运动。必须重新确认机器人状态、选择模式并发送新速度。

## 测试

不依赖 ROS2/SDK/libssh 的核心测试：

```bash
cmake -S hypertron_ros2_bridge -B build-pure \
  -DBUILD_ROS2_BRIDGE=OFF -DBUILD_AGENT=OFF -DBUILD_TESTING=ON
cmake --build build-pure -j
ctest --test-dir build-pure --output-on-failure
python3 -m pytest hypertron_ros2_bridge/test/test_package_contract.py -q
```

有 ROS2 的 PC 上再执行 `colcon build` 和 `colcon test`；机器人或对应架构构建机上用厂家库执行 agent-only 构建。不要把“纯核心测试通过”表述成“实机控制已验证”。

## 已知厂家待确认项

- 手册第 24–26 页未明确 UDP 6100/6101 字节序和 packing。
- 手册第 26 页未明确 6101 位置、四元数固定点比例与坐标方向；默认 `scale_verified: false`。
- 手册第 24 页未明确 UDP 6000 与 H.264 NAL/访问单元边界。
- 手册第 23–26 页和 ASTRALL SDK 1.0.7 没有关节控制/状态 API。`/joint_commands` 只拒绝，`/joint_states` 不制造数据。
- SSH 管理 IP、用户名、机器人发行版和启动权限需要按厂家实机信息修改。

代码中的 `// TODO:参考手册第X页补充实现` 与这些缺口一一对应，厂家提供信息后应补抓包回归测试并提升协议版本。

## 业界参考

- 宇树官方 ROS2 仓库：[unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)。本工程借鉴其“本机 ROS 图直接暴露控制/状态接口”的操作体验，但没有复制其 DDS 或消息协议。
- 云深处开源 ROS 接口参考：[DeepRoboticsLab/Lite3_ROS](https://github.com/DeepRoboticsLab/Lite3_ROS)。本工程采用相似的状态反馈与运动命令边界，但传输层针对 SW01 设计。
- libssh 官方教程：[The Tutorial](https://api.libssh.org/stable/libssh_tutor_guided_tour.html) 与 [channels](https://api.libssh.org/stable/libssh_tutor_shell.html)。实现使用非 PTY exec channel、known_hosts 校验和分离 stderr。
- OpenSSH 客户端配置参考：[ssh_config(5)](https://man.openbsd.org/ssh_config)。
- ROS2 QoS 参考：[ROS 2 Humble QoS settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)。传感器使用 SensorDataQoS，状态使用 Reliable KeepLast(10)。

与通用 “robot dog SSH ROS2 bridge C++ example” 相比，本实现额外使用版本化帧、CRC、能力协商、相关 ACK、有界队列和双 deadman；这避免把 TCP 字节流误当消息边界，也避免断线后回放旧运动指令。
