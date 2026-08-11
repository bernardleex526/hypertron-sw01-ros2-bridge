# HTBR v1 桥接协议

HTBR 是本工程在 SSH exec channel 的 stdout/stdin 上使用的二进制帧协议，不是厂家 UDP 3600 私有协议。所有多字节整数和 IEEE-754 浮点数均为网络字节序；字符串表示为 `u16 长度 + UTF-8 字节`，不发送 C/C++ 裸结构体。

## 帧头

固定头长 28 字节，随后紧跟 payload：

| 偏移 | 长度 | 字段 |
|---:|---:|---|
| 0 | 4 | ASCII `HTBR` |
| 4 | 1 | 协议版本，当前为 1 |
| 5 | 1 | 消息类型 |
| 6 | 2 | flags，v1 必须为 0 |
| 8 | 4 | sequence |
| 12 | 4 | payload 长度 |
| 16 | 8 | 发送端 monotonic time，纳秒 |
| 24 | 4 | CRC-32/ISO-HDLC |

CRC 覆盖“CRC 字段清零后的完整头 + payload”。接收端还检查 magic、版本、类型、flags、最大 payload、保留字段、有限浮点值和 payload 精确长度；任何损坏都终止当前 SSH channel，避免在失去消息边界后继续控制机器人。

## 消息类型与 payload

| 类型 | 值 | 方向 | payload（按顺序） |
|---|---:|---|---|
| HELLO | `0x01` | PC→agent | `min_version u8, max_version u8, reserved u16, capabilities u32, nonce u32` |
| HELLO_ACK | `0x02` | agent→PC | `version u8, reserved u8+u16, capabilities u32, nonce u32, sdk_version text` |
| PING/PONG | `0x03/0x04` | 双向 | 空 |
| CMD_VELOCITY | `0x11` | PC→agent | `vx, vy, vyaw`，各 `f32` |
| CMD_MODE | `0x12` | PC→agent | `ASTRALL mode u16` |
| CMD_ESTOP | `0x13` | PC→agent | `engage u8`，只能为 0/1 |
| CMD_JOINT | `0x14` | PC→agent | v1 保留；当前必定返回 FeatureUnavailable |
| IMU | `0x21` | agent→PC | `device_time u64, accel[3] f32, gyro[3] f32, quaternion[4] f32` |
| SPORT | `0x22` | agent→PC | `device_time u64, wheel_speed[4] f32, sport_status u16` |
| ODOMETRY | `0x23` | agent→PC | `device_time u64, position[3] f64, quaternion[4] f64` |
| ROBOT_STATE | `0x24` | agent→PC | 连接/急停/相机/比例标志、系统/错误/警告/运动状态、电池、轮速、最后速度序号与错误文本 |
| CAMERA_H264 | `0x25` | agent→PC | `stream_id u32, agent_receive_time u64, datagram_sequence u32, H.264 bytes` |
| ACK | `0x7E` | agent→PC | `request_sequence u32, vendor_result u16, text` |
| ERROR | `0x7F` | agent→PC | `request_sequence u32, BridgeError u16, text` |

能力位依次为 IMU、SPORT、ODOMETRY、CAMERA、JOINT、SYSTEM_STATE。未定义位必须为零；PC 校验 HELLO_ACK nonce，且在厂家提供关节协议前拒绝宣称 JOINT 能力的 agent。

## 安全与队列语义

- CMD_ESTOP 走最高优先级队列；CMD_VELOCITY 使用 latest-value mailbox，不积压旧速度；其他命令使用有界 FIFO。
- agent 的控制响应、RobotState、里程计、IMU/运动和相机使用相互独立的有界队列；视频队列只保留最近数据。
- 模式命令串行执行，只有 `AstrallGetSportStatus` 到达手册对应状态才 ACK；超时或急停返回 ERROR。
- SSH 断开时双方清空尚未发送的命令，重连后必须重新 HELLO、选择模式并发送新速度。
- PC 允许最长 65 秒的 agent/SDK 初始化期；收到第一个 PONG 后切换为默认 500 ms 应用心跳超时。

