# Hypertron SW01 ROS 2 直连驱动

本包在 PC（x86_64 / ROS2 Humble）上通过官方 **ASTRALL SDK 1.0.7** 直连
Hypertron SW01 机器人：控制链路走 SDK（UDP 3600），LiDAR 点云/里程计走旁路
UDP 6100/6101。**无 SSH、无 HTBR 帧、无机器人端 agent**，整条链路只在 PC 上的
单个 `hypertron_driver` 节点内。

> ⚠️ **安全红线**：运动会伤人或损坏设备。首次联调必须架空或稳定支撑，实体急停
> 随时可达，遥控器在场可随时抢权。软件 `/emergency_stop` 不是物理安全链路的
> 替代品。本仓库通过代码级测试与审查（含 Oracle 双重评审、ASan/UBSan、TSAN
> 零竞态），但不构成实机运动安全认证。

## 架构

```
ROS 2 应用
  ├─ /cmd_vel /robot_mode /joint_commands ──────────────┐
  ├─ /emergency_stop /control_authority (服务)           │
  └─ /imu/data /odom /robot_state                       │
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

## 1. 主机网络前提

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

## 2. 依赖与安装

- ROS 2 Humble（桌面版）。
- 厂商 ASTRALL SDK 1.0.7 C++（x86_64）：默认路径
  `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++`，含 `include/interface.h` 与
  `lib/linux/x86_64/libASTRALL_SDK.so`。其他机器用
  `-DASTRALL_SDK_ROOT=<路径>`（或环境变量）覆盖。
- 建图链路额外需要（导航骨架引用）：
  ```bash
  sudo apt install ros-humble-pointcloud-to-laserscan
  ```

## 3. 构建

### 纯 CMake（无 ROS，验证核心逻辑与单元测试）

```bash
cmake -S . -B build-pure -DBUILD_ROS2_BRIDGE=OFF -DBUILD_TESTING=ON
cmake --build build-pure -j$(nproc)
ctest --test-dir build-pure --output-on-failure
```

### ROS（直连驱动节点）

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build-ros \
  -DASTRALL_SDK_ROOT=/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++ \
  -DBUILD_TESTING=ON
cmake --build build-ros -j$(nproc)
ctest --test-dir build-ros --output-on-failure
```

### colcon（推荐，配合 ros2_ws）

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hypertron_ros2_bridge
source install/setup.bash
```

`ASTRALL_SDK_ROOT` 默认为上述本机路径；其他机器用
`colcon build --packages-select hypertron_ros2_bridge --cmake-args -DASTRALL_SDK_ROOT=<路径>`。

## 4. 运行

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch hypertron_ros2_bridge driver.launch.py
```

节点名 `hypertron_driver`，可执行名 `hypertron_driver_node`。**无网络时节点不
崩溃**：preflight 失败会以退避节奏持续重试并告警，全程零致动。参数见
`config/driver_config.yaml`（非法参数会使节点启动失败并给出明确报错）。

## 5. ROS 接口

| 方向 | 名称 | 类型 / QoS | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `geometry_msgs/Twist`，KeepLast(1) reliable | vx/vy/vyaw 各分量钳位 ±1；NaN/Inf 一律拒绝 |
| 订阅 | `/robot_mode` | `std_msgs/String` | damping/stand/down/move/auto_charge/exit_charge/recover/recovery（大小写不敏感） |
| 订阅 | `/joint_commands` | `sensor_msgs/JointState` | **一律拒绝**：SDK 1.0.7 无关节接口（拒绝计数进 /robot_state） |
| 发布 | `/imu/data` | `sensor_msgs/Imu`，SensorDataQoS | 含可配协方差、四元数顺序（默认 xyzw） |
| 发布 | `/odom` | `nav_msgs/Odometry` | **仅诊断**：IMU 的 odomX/odomY 映射，twist 全 0 |
| 发布 | `/robot_state` | `hypertron_ros2_bridge/RobotState` | 连接/授权/急停/系统/电池/模式/轮速/错误（2Hz + 状态变化即时） |
| 发布 | `/points` | `sensor_msgs/PointCloud2`（PointXYZRGBA），SensorDataQoS | UDP 6100 点云，整帧重组后发布；**比例未实机标定** |
| 发布 | `/odom_lidar` | `nav_msgs/Odometry`，SensorDataQoS | UDP 6101 里程计；**未标定，不发布任何 TF** |
| 服务 | `/emergency_stop` | `std_srvs/SetBool` | true=锁存软件急停+零速+阻尼；false=仅清除锁存 |
| 服务 | `/control_authority` | `std_srvs/SetBool` | true=申请 SDK 控制权；false=交还遥控器 |

不发布 `/joint_states`；默认不发 odom→base TF（`odom.publish_tf=false` 且
`odometry.scale_verified=false`）。

## 6. 完整实机 SOP（按阶段执行，禁止跳阶段）

**进入任何致动阶段前必须满足：机器人架空/稳定支撑、实体急停可达、遥控器在
场。** 详细标定矩阵与记录表见
[docs/REAL_MACHINE_CALIBRATION.md](docs/REAL_MACHINE_CALIBRATION.md)。

### 阶段 A：网络与只读联调（零致动）

1. 接网线、机器人上电，确认路由（见第 1 节）。
2. 启动驱动，只读观察：
   ```bash
   ros2 topic list
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
（packed 68B / aligned 80B）、6100 布局、0xAA55/0xFF00 头尾、点坐标比例
（默认 int32×1e-3）、odom 位置比例（默认 int64×1e-6）、姿态四元数顺序
（默认 xyzw）、轴向、包序号基（0/1）、单/双雷达、6101 参考系、设备时间戳、
丢包率。**涉及推动机器人的测量必须在安全三前提下进行。**

### 阶段 C：参数回填与复测

按实测值更新 `config/driver_config.yaml`（`lidar.point_position_scale`、
`lidar.odom_position_scale`、`lidar.odom_quaternion_scale`、`lidar.frame_id`、
`odom_lidar.parent_frame/child_frame` 等），重启驱动复测：静止 60s 漂移、
直线误差（建议 ±2%）、旋转误差（建议 ±2°）达标后才谈定位。

### 阶段 D：手动控制权 + 低速运动验证

```bash
ros2 service call /control_authority std_srvs/srv/SetBool "{data: true}"
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: 'stand'}"
ros2 topic pub --once /robot_mode std_msgs/msg/String "{data: 'move'}"
# 低速小位移，方向与预期一致后再增大速度
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

顺序必须完整：明确零速 → ROS deadman（停发 `/cmd_vel` 100ms 自动归零）→
软件急停（`ros2 service call /emergency_stop std_srvs/srv/SetBool "{data: true}"`）
→ 物理急停 → 断网线观察重连与零致动 → 遥控器抢权（驱动不得循环抢回）。

### 阶段 E：建图（SLAM 骨架）

```bash
# 标定 base_link→lidar 外参并回填 config/laserscan_converter.yaml 后：
ros2 launch hypertron_ros2_bridge mapping.launch.py enable_tf_skeleton:=true
```

> ⚠️ `enable_tf_skeleton` 默认 **false**；其静态 TF 参数为占位值
> (0,0,0,0,0,0)，**未标定外参前严禁开启**。建图期间保持低速，保存地图并做
> 静止漂移与地图一致性验收。

### 阶段 F：导航（Nav2 骨架）

```bash
ros2 launch hypertron_ros2_bridge navigation.launch.py map:=<绝对路径>/map.yaml
```

（需同时运行 mapping.launch 由 SLAM 提供 map→odom 定位，或后续加 AMCL。）
低速验收：点到点、避障、断流时导航感知超时（Nav2 应停，而不是用陈旧数据）。

### 紧急情况

| 情形 | 动作 |
|---|---|
| 任何时候异常运动 | 先拍物理急停，再查日志 |
| 遥控器在场 | 遥控器抢权优先级高于 SDK，驱动不会抢回 |
| 断网/断连 | 驱动自动重连但**不恢复**授权/模式/速度，速度链路保持零 |
| 软件急停 | `/emergency_stop` 锁存；解除后不重放任何速度/模式 |

## 7. 建图与导航（骨架说明）

- `launch/mapping.launch.py`：driver + `pointcloud_to_laserscan`
  （`/points`→`/scan`）+ `slam_toolbox`（online async，mode=mapping）。
- `launch/navigation.launch.py`：封装 `nav2_bringup`，速度保守
  （max_vel_x 0.3 / max_vel_theta 0.5），costmap 订阅 `/scan` 与 `/points`。
- `config/nav2_params.yaml` 中 `robot_radius`/`inflation_radius` 等为占位值，
  实机测量后回填。
- odom→base TF 在里程计标定完成前**不发布**（由后续 robot_localization 或
  标定后的里程计源提供）。

## 8. 限制

- **无关节接口**：`/joint_commands` 一律拒绝且不下行，无 `/joint_states`。
- **里程计与点云比例未实机标定**：`odometry.scale_verified=false`（默认），
  标定前 `/odom`、`/odom_lidar`、`/points` 均不可用于定位闭环；默认不发 TF。
- **相机 H.264（UDP 6000）属后续范围**：本驱动不解析相机流。
- 实机运动安全认证未完成。

## 9. 测试与验证

```bash
# 单元/集成（纯 CMake 6 项，含 runtime 44 例、lidar 50 例）
ctest --test-dir build-pure --output-on-failure
# ROS（10 项，含节点 15 例、合同 pytest 13 项）
ctest --test-dir build-ros --output-on-failure
# 并发门禁（clang TSAN，期望 0 race；ASLR 兼容用 setarch -R）
cmake -S . -B build-tsan -DBUILD_ROS2_BRIDGE=OFF -DBUILD_TESTING=ON \
  -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -O1 -g"
cmake --build build-tsan -j$(nproc)
setarch -R build-tsan/test_direct_driver_runtime --gtest_brief=1
```

## 10. 故障排查

| 现象 | 处理 |
|---|---|
| 持续 `network preflight failed` | eno1 载波/地址/路由（第 1 节）；代理未排除 10.18.0.0/24 |
| `/robot_state.sdk_linked=false` | SDK 初始化失败；查日志中 init/subscribe 报错与心跳 |
| 无 `/points` | 检查 `subscriptions.lidar_enabled`、`lidar.source_ip`（默认 10.18.0.100）、端口占用 |
| 节点启动即退（InvalidParametersException） | 参数非法：端口/时间/容量/频率/协方差范围，按报错修正 `driver_config.yaml` |
| `ros2 pkg executables` 有多个可执行 | 旧 `sw01_ros2_driver` 未加 COLCON_IGNORE，会双进程抢 UDP 3600 |

## 参考资料

- [SW01 手册摘要](SW01_MANUAL_NOTES.md)：SDK API、返回码、UDP 6000/6100/6101 结构
- [实机标定 Runbook](docs/REAL_MACHINE_CALIBRATION.md)：13 项标定矩阵与记录表
- [实现与安全评审](REVIEW.md)：任务状态、验证矩阵、遗留项
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/index.html)
