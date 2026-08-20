# Hypertron SW01 软件开发手册提要（直连驱动视角）

来源：`Hypertron-SW01 软件开发手册-中文版.pdf`，ASTRALL SDK 1.0.7，30 页。本文件是实现索引，不替代厂家原手册；页码按 PDF 页码。本驱动采用 PC 直连 SDK 模式，无机器人端 agent、无 SSH、无 HTBR。

## 网络与进程模型

- 第 4 页给出的默认网络为机器人 `10.18.0.100`、SDK 设备端 `10.18.0.200`，SDK 本地 UDP 端口 `3600`。本驱动只通过厂家 `libASTRALL_SDK.so` 使用该 UDP 3600 报文，不重做该私有协议。
- 第 23–26 页的数据流使用 UDP：RGB H.264 为 `6000`，LiDAR 点云为 `6100`，LiDAR 里程计为 `6101`。
- ASTRALL SDK 的 C++ 头文件是 `interface.h`；Linux 库同时提供 x86_64 与 ARM64 版本。直连驱动使用 x86_64 版本，运行在 PC 上。

## 返回码

| 名称 | 值 | 含义 |
|---|---:|---|
| `ASTRALL_RES_FAILED` | `0x8004` | 执行、申请或订阅失败 |
| `ASTRALL_RES_TIMEOUT` | `0x8005` | 超时 |
| `ASTRALL_RES_RUNNING` | `0x8006` | 执行中 |
| `ASTRALL_RES_SUCCESSED` | `0x8008` | 成功（厂家拼写保留） |
| `ASTRALL_RES_INVALID_PARAM` | `0x8010` | 参数无效 |
| `ASTRALL_RES_NOT_INIT` | `0x8011` | SDK 未初始化 |
| `ASTRALL_RES_RC_NORELEASE` | `0x8020` | 遥控器未释放控制权 |
| `ASTRALL_RES_BEEN_OBTAINED` | `0x8021` | 其他设备已持有控制权 |
| `ASTRALL_RES_WITHOUT_AUTH` | `0x8022` | 当前没有控制权 |

## 关键 API 与生命周期

以下签名来自随 SDK 提供的 C++ `interface.h`：

```cpp
uint16_t AstrallSdkInit(AstrallConfig& cfg, uint32_t timeout = 60000);
void AstrallSdkDeinit();
uint16_t AstrallHeartbeat(uint32_t timeout = 20);
uint16_t AstrallGetSystemStatus(AstrallSystemStatus& status);
uint16_t AstrallGetPowerStatus(AstrallPowerStatus& power);
uint16_t AstrallGetSportStatus();
uint16_t AstrallGetDeviceInfo(AstrallDeviceInfo& info, uint32_t timeout = 20);
uint16_t AstrallGetImuData(AstrallImuData& data);
uint16_t AstrallSubscriptionData(
    AstrallSubscribeTopicId topicId, AstrallSubscribeFreq freq,
    std::function<void(void* data, uint16_t len)> cb, uint32_t timeout = 20);
uint16_t AstrallSportModeControl(AstrallSportModeCmd mode,
                                 uint32_t timeout = 20);
uint16_t AstrallAuthControl(AstrallAuth auth, uint32_t timeout = 20);
uint16_t AstrallMove(float vx, float vy, float vyaw,
                     uint32_t timeout = 20);
uint16_t AstrallLightControl(AstrallLightCmd cmd, uint32_t timeout = 20);
uint16_t AstrallSendMessage(char* data, uint16_t len);
```

生命周期必须为 init →（订阅/申请控制权/运动）→ deinit。`AstrallSdkInit` 通过
`AstrallConfig::heartbeatCb` 与 `sdkStatusCb` 上报心跳和 `link/ctrlAuthority`；
`AstrallHeartbeat` 是应用层心跳，`AstrallMove` 下发速度，三者是直连驱动核心
生命周期调用的三块。

初始化成功并不代表已取得控制权：`AstrallAuthControl(ASTRALL_AUTH_SDK)`
才申请 SDK 控制权；`ASTRALL_AUTH_SDK` 值为 `1`，`ASTRALL_AUTH_JOYSTICK`
值为 `2`（遥控器）。遥控器抢权后，驱动不循环抢回。`/control_authority` 服务
`true` 表示申请 `ASTRALL_AUTH_SDK`，`false` 表示交还遥控器。

其余关键结构：`AstrallDeviceInfo` 含版本字符串、序列号和 `model u32`；
`AstrallSdkStatus` 是 `link/ctrlAuthority/reserve` 位域；`AstrallJoystickData`
含 64 位时间戳、六轴 `int16 rocker[6]` 及 L1/L2/L3、R1/R2/R3、SL/SR、H 和两位
SW 按键位。当前直连驱动不发布设备信息或遥控器输入，但这些定义保留在厂家适配
边界，后续扩展时不得按网络结构直接发送位域。

## 模式、运动与安全时序

| ROS 字符串 | 厂家命令 | 对应主要稳定状态 |
|---|---:|---:|
| `damping` | `0xA101` | `0xB101` |
| `stand` | `0xA102` | `0xB102` |
| `down` | `0xA103` | `0xB103` |
| `move` | `0xA104` | `0xB104` |
| `auto_charge` | `0xA105` | `0xB107` 起始 |
| `exit_charge` | `0xA106` | `0xB10B` 过程 |
| `recover`（兼容 `recovery`） | `0xA1FF` | `0xB1FF` |

- `AstrallMove(vx, vy, vyaw)` 三个分量均为 `[-1, 1]`，停止必须明确发送 `(0,0,0)`。
- 机器人在运动中约 50 ms 未收到运动指令会自动停止，因此驱动以 50 Hz
  （`motion_refresh_period_ms=20`）刷新。
- 推荐 `AstrallHeartbeat` 为 10 Hz。驱动实现上对每次心跳采用双段超时：单次
  心跳调用超时 `heartbeat_call_timeout_ms=500`，连续失败
  `heartbeat_max_failures=1` 即判链路异常。约 500 ms 无心跳时清除指令并锁定
  站立；再失联则清除连接与控制权，无其他控制器时自主趴下。
- 系统错误码非零时，机器人进入被动安全状态，桥接层拒绝运动。

## 系统、电池和订阅数据

`AstrallSystemStatus` 包含 `AstrallSystemCode sysStatus`、32 位 `errorCode`
与 `warnCode`。系统状态从初始化、待机、运行、警告、错误到关机；错误位包括
电机版本/通信/状态/过温、STO、IMU、低电、过载、MBUS 和相机丢失。警告位包括
过温、遥控器锁定/介入/丢失、通信不稳、低电、过载及 SDK 掉线。

`AstrallPowerStatus`：

- `float soc`：剩余电量百分比；
- `float temp`：电池温度；
- `float voltage`：电池电压；
- `uint16_t cycleCount`：循环次数；
- `uint16_t charged`：充电状态。

订阅 topic ID 与频率：

| Topic | ID | 数据/传输 |
|---|---:|---|
| IMU | `0x0001` | `AstrallImuData` 回调 |
| 运动 | `0x0002` | `AstrallSportData` 回调 |
| RGB | `0x0003` | UDP 6000 H.264 |
| LiDAR | `0x0005` | UDP 6100/6101 |
| 遥控器 | `0x0010` | `AstrallJoystickData` 回调 |

频率枚举支持关闭、1、25、50、125、250 Hz。直连驱动默认 IMU 与 sport 均订阅
50 Hz（`subscriptions.imu_frequency_hz=50`、`sport_frequency_hz=50`）。

`AstrallImuData` 包含 64 位时间戳、`accelerometer[3]`（m/s²）、`gyroscope[3]`
（rad/s）、`quaternion[4]`、pitch/roll/yaw 和 odomX/odomY。`AstrallSportData`
包含 64 位时间戳和 `wheelSpeed[4]`。回调内存由 SDK 管理，本驱动在回调返回前
逐字段复制。

## UDP 6100 点云

手册第 24–25 页描述：

- 头 `0xAA55`、尾 `0xFF00`；
- 64 位时间戳、帧序号、数据序号、点数；
- 每包最多 50 点；
- 单点为三个 `int32` 坐标和四个 `uint32` RGBA 分量。

### AstrallPosCloudData 结构（直连驱动实现）

直连驱动在 `include/.../lidar_stream.hpp` 实现了 UDP 6100 解析与多包组帧
（`ParsePointCloudPacket` + `PointCloudFrameAssembler`）：

- **SOF/TAIL**：头 `0xAA55`、尾 `0xFF00`。
- **两种布局**：Naturally-packed 头（20 字节头：SOF、64 位时间戳、u32 帧
  总点数 total、u32 数据序号 index、u16 posNum）与 type-aligned 头（28 字节头）。
- **尾部两种形态**：动态尾（恰好 `posNum` 个点后接尾标记）与固定 50 点尾
  （`posNum` 个有效点，其余忽略），解析均校验 0xAA55/0xFF00 与长度。
- 点宽 28 字节：x/y/z `int32` + rgba `uint32`（字节序原样保留，不臆断通道顺序）。
- 组帧按 `total`（帧总点数）与 `index`（本包起始点偏移，实测 0、50、100、...）
  精确覆盖 `[0, total)` 才发布；单包 ≤50 点约束由解析与 `posNum` 校验承担。
- **坐标比例**：`point_position_scale` 默认 `1e-3`（`int32` × 1e-3 → 米），
  **未实机标定**，标定前不得当作物理真值。

## UDP 6101 里程计

手册第 26 页描述：头、64 位纳秒时间戳、三个 `int64` 位置、四个 `int64`
四元数、尾。手册未给位置/四元数固定点比例和坐标方向。

### AstrallOdometryData 结构（直连驱动实现）

`ParseOdometryPacket` 实现：先试 packed 68 字节头（SOF `0xAA55`、64 位
`int64` 时间戳、x/y/z `int64`、qx/qy/qz/qw `int64`、尾 `0xFF00`），再试
type-aligned 80 字节头。

- **比例默认值**：`odom_position_scale` 默认 `1e-6`（`int64` × 1e-6 → 米）、
  `odom_quaternion_scale` 默认 `1.0`（四元数分量归一化前缩放）。
- **未实机标定**：以上比例与坐标方向均未实机抓包确认。`/odom_lidar` 为原始
  发布（frame `odom` → child `lidar`），不发布 TF；
  `odometry_scale_verified`（YAML `odometry.scale_verified`）默认 `false`，
  标定前不得把数值当作已验证的物理真值。

## RGB 与关节接口限制

手册第 23–24 页只说明 UDP 6000 发送 H.264，没有说明一个 UDP 数据报与
NAL/访问单元的边界关系。相机旁路数据流属后续范围，直连驱动默认不启用。

ASTRALL SDK 1.0.7 和手册第 23–26 页均没有关节角控制或关节状态结构。因此
`/joint_commands` 存在但安全拒绝且不下行，`/joint_states` 不发布伪造值。
待厂家提供低层 SDK、关节命名、单位、限位和反馈后再实现。
