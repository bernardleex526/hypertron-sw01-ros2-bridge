# Hypertron SW01 ROS 2 直连驱动（更新版）

本包在 PC（x86_64 / ROS2 Humble，即 ROS 2 Humble）上通过官方 **ASTRALL SDK 1.0.7** 直连
Hypertron SW01 机器人：控制链路走 SDK（UDP 3600），LiDAR 点云/里程计走旁路
UDP 6100/6101。**无 SSH、无 HTBR、无机器人端 agent**。退役的 HTBR 协议编解码
代码已从构建中删除，安全控制器所需的值类型直接定义在
`hypertron_ros2_bridge/robot_controller.hpp`。

> ⚠️ **安全红线**：运动会伤人或损坏设备。首次联调必须架空或稳定支撑，实体急停
> 随时可达，遥控器在场可随时抢权。软件 `/emergency_stop` 不是物理安全链路的
> 替代品。代码测试与静态审查不构成实机运动安全认证。

---

## 0. 本版本改动与状态

本版本完成了以下代码修复与清理（**尚未在真实 ROS 2 + ASTRALL SDK 环境重新构建，
实机前必须先跑第 6 节测试**）：

- 修复 `frames.imu/odom/base` 参数声明后未生效的问题；
- 修复 authority 丢失/系统错误清除控制器模式状态后，旧模式请求 future
  可能永久悬挂或被新请求覆盖的问题；
- 模式切换前若机器人仍在运动，先发送一次显式零速再执行 `set_mode`；
- `/robot_state` 不再被较旧的 snapshot 覆盖 link/authority，并在断线/告警时
  立即发布；初始状态构造完成即发布，QoS 改为 `transient_local`；
- `/robot_state` 过滤非有限 battery/wheel-speed，并补齐 `header.frame_id`；
- 断线时清除旧 odom→base TF 位姿缓存；
- 增加 IPv4、LiDAR 端口重复、点云内存总预算校验，以及
  motion-refresh/deadman、heartbeat 超时关系告警；
- `send_zero_and_damping()` 在零速失败时保留“待重试零速”标记；
- `ASTRALL_SDK_ROOT` 环境变量在全新 configure 时生效；
- 修复 `mapping.launch.py` 中 `/scan_out` 无效 remap（Humble 节点本就直接发布
  `/scan`）；
- 删除退役 HTBR `protocol_handler` 源码/头文件/测试、无用的
  `reject_joint_command` 控制器路径、`last_sent_velocity_` 等死代码；
- `package.xml`/CMake 补齐 `tf2_msgs`、`launch`/`launch_ros` 依赖。

仍待实机或后续处理的厂商协议问题见 [第 9 节“已知未解决问题”](#9-已知未解决问题)。

## 1. 架构

```
ROS 2 应用
  ├─ /cmd_vel /robot_mode /joint_commands ──────────────┐
  ├─ /emergency_stop /control_authority (服务)           │
  └─ /imu/data /odom /robot_state                        │
        /points /odom_lidar                             │
            ▲                                            │
            │ HypertronDriverNode                        │
            ├─ DirectDriverRuntime（worker：preflight→init→订阅→心跳/轮询/运动循环）
            ├─ DirectAstrallSdk（唯一 include 厂商 interface.h 的 TU）
            └─ LidarStreamReceiver（UDP 6100/6101 收流解析 + 有界帧重组）
                 │                                        │
                 ▼                                        ▼
        libASTRALL_SDK.so (x86_64) ── UDP 3600 ──► SW01（10.18.0.100）
```

安全核心原则：启动非致动；重连绝不恢复授权/模式/速度；断连即清控制器状态；
软件急停锁存不随断连清除、解除不重放；关闭时仅在自己持有授权时发零速+阻尼。

## 2. 主机网络前提

- PC 网卡配置 `10.18.0.200/24`（参数 `interface="eno1"`、`host_address`）。
- 与机器人 `10.18.0.100` 三层可达（同网段直连）。
- 若使用代理（如 mihomo），必须在代理规则排除 `10.18.0.0/24`，否则 SDK 的
  UDP 3600 会被劫持。
- SDK 库内**硬编码**目标 `10.18.0.100:3600`，无运行时配置项。

```bash
ip -brief addr show eno1          # 应有 10.18.0.200/24
ip route get 10.18.0.100          # 应走 eno1、src=10.18.0.200
ping -c 3 10.18.0.100
```

## 3. 依赖与安装

- ROS 2 Humble（桌面版）。
- 厂商 ASTRALL SDK 1.0.7 C++（x86_64）：需包含 `include/interface.h` 与
  `lib/linux/x86_64/libASTRALL_SDK.so`。全新 configure 时可用环境变量
  `ASTRALL_SDK_ROOT` 或 `-DASTRALL_SDK_ROOT=<路径>` 指定；默认仍为原调试机
  路径 `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++`。
- 建图链路额外需要：
  ```bash
  sudo apt install ros-humble-pointcloud-to-laserscan
  ```

## 4. 构建

### 纯 CMake（无 ROS，验证核心逻辑与单元测试）

```bash
cmake -S . -B build-pure -DBUILD_ROS2_BRIDGE=OFF -DBUILD_TESTING=ON
cmake --build build-pure -j$(nproc)
ctest --test-dir build-pure --output-on-failure
```

纯配置共 5 项 CTest（thread_safe_queue、robot_controller、network_preflight、
direct_driver_runtime、lidar_stream）。

### ROS（直连驱动节点）

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build-ros \
  -DASTRALL_SDK_ROOT=/path/to/ASTRALL_SDK_1.0.7/C++ \
  -DBUILD_TESTING=ON
cmake --build build-ros -j$(nproc)
ctest --test-dir build-ros --output-on-failure
```

ROS 配置共 9 项 CTest（上述 5 项 + package_contract pytest、astrall_sdk、
direct_astrall_sdk、driver_node）。

### colcon（推荐）

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hypertron_ros2_bridge
source install/setup.bash
```

## 5. 运行

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hypertron_ros2_bridge driver.launch.py
```

节点名 `hypertron_driver`，可执行名 `hypertron_driver_node`。**无网络时节点不
崩溃**：preflight 失败会以退避节奏持续重试并告警，全程零致动。参数见
`config/driver_config.yaml`；参数范围/交叉关系错误会在启动时失败或告警。

## 6. 测试与验证（实机前必做）

```bash
# 纯 CMake
cmake -S . -B build-pure -DBUILD_ROS2_BRIDGE=OFF -DBUILD_TESTING=ON
cmake --build build-pure -j$(nproc)
ctest --test-dir build-pure --output-on-failure

# ROS
source /opt/ros/humble/setup.bash
cmake -S . -B build-ros -DASTRALL_SDK_ROOT=<路径> -DBUILD_TESTING=ON
cmake --build build-ros -j$(nproc)
ctest --test-dir build-ros --output-on-failure
```

本版本修改尚未在真实 ROS/SDK 环境执行上述构建；合并后、实机前必须先完整跑
通。并发门禁（如需要）：

```bash
cmake -S . -B build-tsan -DBUILD_ROS2_BRIDGE=OFF -DBUILD_TESTING=ON \
  -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -O1 -g"
cmake --build build-tsan -j$(nproc)
setarch -R build-tsan/test_direct_driver_runtime --gtest_brief=1
```

## 7. ROS 接口

| 方向 | 名称 | 类型 / QoS | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist`，KeepLast(1) reliable | vx/vy/vyaw 各分量钳位 ±1；NaN/Inf 一律拒绝 |
| 订阅 | `/robot_mode` | `std_msgs/String` | damping/stand/down/move/auto_charge/exit_charge/recover/recovery（大小写不敏感） |
| 订阅 | `/joint_commands` | `sensor_msgs/JointState` | **一律拒绝**：SDK 1.0.7 无关节接口（拒绝计数进 /robot_state） |
| 发布 | `/imu/data` | `sensor_msgs/Imu`，SensorDataQoS | 含可配协方差、四元数顺序（默认 xyzw） |
| 发布 | `/odom` | `nav_msgs/Odometry`，SensorDataQoS | **仅诊断**：IMU 的 odomX/odomY 映射，twist 全 0 |
| 发布 | `/robot_state` | `RobotState`，KeepLast(10) reliable + transient_local | 连接/授权/急停/系统/电池/模式/轮速/错误；连接期间约 2Hz，状态变化和断线告警即时发布 |
| 发布 | `/points` | `sensor_msgs/PointCloud2`（PointXYZRGBA），SensorDataQoS | UDP 6100 点云，整帧重组后发布；**比例未实机标定** |
| 发布 | `/odom_lidar` | `nav_msgs/Odometry`，SensorDataQoS | UDP 6101 里程计；**未标定，不发布任何 TF** |
| 服务 | `/emergency_stop` | `std_srvs/SetBool` | true=锁存软件急停+零速+阻尼；false=仅清除锁存 |
| 服务 | `/control_authority` | `std_srvs/SetBool` | true=申请 SDK 控制权；false=交还遥控器 |

不发布 `/joint_states`；默认不发 odom→base TF（`odom.publish_tf=false` 且
`odometry.scale_verified=false`）。

## 8. 完整实机 SOP（按阶段执行，禁止跳阶段）

**进入任何致动阶段前必须满足：机器人架空/稳定支撑、实体急停可达、遥控器在
场。** 详细标定矩阵见
[docs/REAL_MACHINE_CALIBRATION.md](docs/REAL_MACHINE_CALIBRATION.md)。

### 阶段 A：网络与只读联调（零致动）

1. 接网线、机器人上电，确认路由（见第 2 节）。
2. 启动驱动，只读观察：
   ```bash
   ros2 topic list
   ros2 topic echo /robot_state --once    # 初始状态会立即发布
   ros2 topic echo /robot_state --once    # sdk_linked: true 为链路正常
   ros2 topic hz /imu/data                # 默认 50Hz
   ros2 topic hz /points                  # 有数据=LiDAR 开流成功
   ```
   验收：`sdk_linked=true`、`control_authority=false`、`emergency_stop=false`、
   `/points` 与 `/odom_lidar` 有数据。

### 阶段 B：抓包与协议标定（决定一切上层用法）

```bash
sudo tcpdump -i eno1 -w /tmp/lidar6100.pcap udp port 6100 -c 2000
sudo tcpdump -i eno1 -w /tmp/odom6101.pcap udp port 6101 -c 500
```

按 `docs/REAL_MACHINE_CALIBRATION.md` 的 13 项矩阵逐项确认：6101 布局
（packed 68B / aligned 80B）、6100 布局、0xAA55/0xFF00 头尾、点坐标比例、
odom 位置比例、四元数顺序、轴向、包序号基、单/双雷达、6101 参考系、设备
时间戳、丢包率。**涉及推动机器人的测量必须在安全三前提下进行。**

> 当前代码的 6100 字段语义和 RGBA 处理仍是未实机验证的假设，见第 9 节；
> 标定未完成前，`/points`、`/odom_lidar`、`/scan` 都不得进入定位/导航闭环。

### 阶段 C：参数回填与复测

按实测值更新 `config/driver_config.yaml`：

- `lidar.point_position_scale` / `lidar.odom_position_scale` /
  `lidar.odom_quaternion_scale`；
- `lidar.frame_id`、`odom_lidar.parent_frame/child_frame`；
- `frames.imu/odom/base`（本版本起真正生效）。

重启驱动复测：静止 60s 漂移、直线误差（建议 ±2%）、旋转误差（建议 ±2°）达标
后才谈定位。

### 阶段 D：手动控制权 + 低速运动验证

```bash
ros2 service call /control_authority std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: 'stand'}"
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: 'move'}"
# 低速小位移，方向与预期一致后再增大速度
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

顺序必须完整：明确零速 → ROS deadman（停发 `/cmd_vel` 100ms 自动归零）→
软件急停（`/emergency_stop` SetBool true）→ 物理急停 → 断网线观察重连与零
致动 → 遥控器抢权（驱动不得循环抢回）。

> 软件急停会发送 damping。解除急停锁存后不会重放任何速度/模式；通常需要
> 重新发送 `move` 模式并重新发送速度，这是有意的保守行为。

### 阶段 E：建图（SLAM 骨架）

```bash
# 标定 lidar→base_link 外参并回填 config/laserscan_converter.yaml 后：
# 若使用 /odom_lidar 作为里程计 TF，还需要 enable_odom_tf:=true
ros2 launch hypertron_ros2_bridge mapping.launch.py \
  enable_tf_skeleton:=true enable_odom_tf:=true
```

> ⚠️ `enable_tf_skeleton` 默认 **false**；其静态 TF 参数为占位值
> (0,0,0,0,0,0)，**未标定外参前严禁开启**。注意当前骨架还没有
> odom→base_link 的定位 TF 源；slam_toolbox 需要该 TF，后续需接
> robot_localization 或标定后的里程计源才能实际成图。

### 阶段 F：导航（Nav2 骨架）

```bash
# 独立导航：map_server + AMCL
ros2 launch hypertron_ros2_bridge navigation.launch.py \
  map:=<绝对路径>/map.yaml use_amcl:=true

# 配合 mapping.launch 在线 SLAM 定位
ros2 launch hypertron_ros2_bridge navigation.launch.py use_amcl:=false
```

（独立导航需要标定后的 odom→base_link TF；在线 SLAM 需要同时运行
mapping.launch 提供 map→odom 定位。）
低速验收：点到点、避障、断流时导航感知超时（Nav2 应停，而不是用陈旧数据）。

### 紧急情况

| 情形 | 动作 |
|---|---|
| 任何时候异常运动 | 先拍物理急停，再查日志 |
| 遥控器在场 | 遥控器抢权优先级高于 SDK，驱动不会抢回 |
| 断网/断连 | 驱动自动重连但**不恢复**授权/模式/速度，速度链路保持零 |
| 软件急停 | `/emergency_stop` 锁存；解除后不重放任何速度/模式 |

## 9. 已知未解决问题

以下问题涉及厂商协议字段、频率和比例，本版本**有意未改代码**，必须通过
`interface.h`、SDK 文档和实机抓包确认后再处理：

1. **UDP 6100 字段语义未证实**：当前解析把 offset 10 当作“帧总点数”、
   offset 14 当作“包序号”；手册笔记写的是“帧序号、数据序号”。若实机
   offset 10 是帧序号，现有组帧算法将无法发布 `/points`。
2. **UDP 6100 RGBA 发布仍只使用第一个 uint32**：解析层已保留全部 4 个
   uint32 通道（`rgba_channels`），但 `/points` 为兼容 PointXYZRGBA 仍只把
   第一个通道发布为 `rgba`；如需完整颜色/强度映射，需按真实结构在
   `driver_node.cpp` 增加扩展字段。
3. **LiDAR 订阅频率已可配置**：`subscriptions.lidar_frequency_hz` 默认 1 Hz；
   如果 SDK 的订阅频率决定 UDP 6100/6101 推送频率，实机确认后调高即可。
   仍需确认该频率与 `frame_timeout_ms=300` 的配合。
4. **6100/6101 布局与尾部 padding 未证实**：当前只接受精确动态/固定尾长度，
   旧实现曾接受 aligned 尾部 padding；若实机带 padding，需恢复兼容。
5. **6101 四元数顺序已可配置**：新增 `lidar.odom_quaternion_order`（xyzw/wxyz）。
   `odom_quaternion_scale` 仍会被归一化约掉，需在实机标定时确认是否应删除该参数。
6. **recover/recovery 在 error_code≠0 时被拒绝**：若 recovery 的目标场景就是
   系统错误后的恢复，该门禁会使其不可用；需按手册确认。
7. **heartbeat/motion/getter 共用同一 worker**：最坏情况下阻塞型 SDK 调用
   （heartbeat 500ms、无超时的 getter）会中断 20ms 运动刷新并使 stop()
   等待较久。需要 SDK 实测延迟后决定是否拆分线程或收紧超时。
8. **`max_packets_per_frame` 默认已调整为 40000**：可组满 2,000,000 点
   （50 点/包）。实机点云规模确认后仍需按实际值回填。
9. **SLAM/Nav2 仍是待标定骨架**：已增加 `odom_lidar.publish_tf`、
   `mapping.launch.py enable_odom_tf` 和 `navigation.launch.py use_amcl`
   支持，但 `pointcloud_to_laserscan`、slam_toolbox、Nav2 参数仍为占位值，
   且必须完成外参/里程计标定后才能得到可用地图/导航。
10. **相机 H.264（UDP 6000）未实现**；`/joint_commands` 恒拒绝、无
    `/joint_states`。
11. **本版本未做 ROS/SDK 实机构建验证**：修复后需在目标机器重新跑第 6 节
    测试、ASan/UBSan/TSAN 抽查与 launch 存活冒烟。

## 10. 故障排查

| 现象 | 处理 |
|---|---|
| 持续 `network preflight failed` | eno1 载波/地址/路由（第 2 节）；代理未排除 10.18.0.0/24 |
| `/robot_state.sdk_linked=false` | SDK 初始化失败；查日志中 init/subscribe 报错与心跳 |
| 无 `/points` | 检查 `subscriptions.lidar_enabled`、`lidar.source_ip`（默认 10.18.0.100）、端口占用、LiDAR 解析 warning；也见第 9 节未标定问题 |
| 节点启动即退（InvalidParametersException） | 参数非法：端口/时间/容量/频率/协方差/IP/内存预算，按报错修正 `driver_config.yaml` |
| `/robot_state` 收不到 | 本版本已加初始发布与 transient_local；确认 QoS 兼容 |
| `ros2 pkg executables` 有多个可执行 | 旧 `sw01_ros2_driver` 未加 COLCON_IGNORE，会双进程抢 UDP 3600 |

## 参考资料

- [SW01 手册摘要](SW01_MANUAL_NOTES.md)：SDK API、返回码、UDP 6000/6100/6101 结构
- [实机标定 Runbook](docs/REAL_MACHINE_CALIBRATION.md)：13 项标定矩阵与记录表
- [实现与安全评审](REVIEW.md)：任务状态、验证矩阵、遗留项
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/index.html)
