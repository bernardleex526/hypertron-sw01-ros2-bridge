# 实现与安全评审

评审对象为 Hypertron SW01 ROS2/SSH bridge 的 PC 节点、HTBR v1、机器人 agent、ASTRALL SDK 适配、UDP 解析、ROS 数据映射、配置和部署文档。结论：未发现遗留 P0/P1/P2 代码问题；可进入厂家信息补全和实机调试阶段，但不能把本报告当作实机运动安全认证。

## 已处理的主要评审项

| 级别 | 发现 | 处理结果 |
|---|---|---|
| P1 | 仅监听 UDP 6000/6100/6101、未先调用厂家 RGB/LiDAR 订阅 | agent 初始化阶段调用 `AstrallSubscriptionData`，失败即停止启动；测试验证 init→subscribe→acquire 顺序 |
| P1 | 模式调用收到 SDK 返回成功就提前 ACK | 使用独立串行模式 worker，轮询实际 `sport_status`，到达手册稳态才 ACK，超时/急停返回 ERROR |
| P1 | 急停与速度/普通命令共用 FIFO，旧速度可积压 | 急停高优先级队列、速度 latest-value mailbox、普通命令有界 FIFO；突发回归测试验证急停先发且只发送最新速度 |
| P1 | SSH stop 可能在 worker 读取 libssh 时并发释放 session | backend 完全由 worker 持有；stop 先发停止信号、join 后才 disconnect；并发回归测试通过 |
| P1 | agent writer 失败时 reader 可能永久阻塞 | writer 失败立即关闭字节流以唤醒 reader；回归测试验证无需外部 close 即退出 |
| P1 | `last_error` 跨线程数据竞争 | 所有读写由 mutex 保护 |
| P1 | H.264 解码阻塞 SSH 收包线程 | PC 使用容量 2、DropOldest 的独立 camera worker；图像采用 PC ROS 接收时间而非 agent steady-clock 时间 |
| P1 | 可关闭严格主机密钥校验并自动登记未知主机 | 配置为 false 会被拒绝；libssh 只接受已验证 known_hosts，不自动更新 |
| P2 | agent 的状态、传感器、里程计、相机共用队列 | 分离为控制、状态、里程计、传感器和相机队列，writer 按安全优先级调度 |
| P2 | PC `/robot_mode` 可并发覆盖且无完成/超时状态 | PC 同时只允许一个 pending 模式；根据相关 ACK/ERROR 完成，并由 wall timer 清理超时 |
| P2 | CAMERA_H264 wire order 与批准格式不一致 | 固定为 `stream_id/receive_time/datagram_sequence/raw`，加入 golden bytes 测试 |
| P2 | HELLO_ACK 缺少 nonce/能力校验 | 检查会话 nonce、协议版本、保留能力位，并在关节语义未定义时拒绝 JOINT capability |
| P2 | 参数先转无符号后校验，存在负数环绕 | SSH 端口、周期、队列、payload、agent UDP 端口及 u32 字段均先做范围校验 |
| P2 | IMU 配置错误地控制里程计时间戳 | IMU 与 odometry 使用独立 timestamp source；相机使用 PC 接收时间 |
| P2 | ROS 命令 QoS 会积压旧命令 | `/cmd_vel` 和 `/joint_commands` 使用 Reliable KeepLast(1)；模式保持 KeepLast(10) 并由 pending gate 串行化 |
| P2 | SDK 初始化最长 60 秒，但 SSH 500 ms 就判 agent 失活 | 增加 65 秒 agent startup timeout；收到首个 PONG 后才切换到 500 ms steady-state timeout |

## 验证矩阵

- 纯 C++：6 个 CTest target 全部通过，覆盖队列、CRC/分帧/payload、控制门控、agent 生命周期/模式确认、SSH 重连/抢占/并发停止和数据映射。
- Python 合同：3 个 pytest 全部通过，检查交付结构、ROS 接口声明、camera worker 和运维文档。
- Sanitizer：ASan + UBSan 与 `-Wall -Wextra -Wpedantic -Werror` 运行全部纯测试通过。
- ROS2：在 Ubuntu 22.04 / ROS2 Humble 上启用 libssh 0.9.6 和 FFmpeg，节点与自定义消息以 warnings-as-errors 完整编译、链接；`ldd` 确认 libssh/libavcodec/libswscale。
- 厂家 SDK：x86_64 ASTRALL 1.0.7 实库完成 agent-only warnings-as-errors 编译/链接；`ldd` 不含 ROS2 或 libssh。
- ThreadSanitizer：当前 WSL 的 TSAN runtime 在测试进程启动前报 `unexpected memory mapping`，因此无有效 TSAN 结果；并发路径由锁审计、ASan/UBSan 和专门回归测试覆盖。
- ROS graph CLI：该 WSL 中跨进程 DDS discovery 对本节点和 ROS 官方 demo talker 均不可用，因此没有把 `ros2 topic list` 冒烟结果记为通过；需在目标 PC 环境执行 README 中的 graph 验证。

## 厂家/实机门禁

以下不是可安全猜测的实现细节，仍保持显式 TODO 和状态标志：

1. ASTRALL 1.0.7 没有关节角命令/状态 API；`/joint_commands` 拒绝，`/joint_states` 静默。
2. 手册未定义 UDP 6100/6101 的字节序、packing、坐标系和固定点比例；必须抓包标定，完成前 `odometry_scale_verified=false`。
3. 手册未定义 UDP 6000 数据报与 H.264 NAL/访问单元边界；FFmpeg parser 可容忍分片，但需实机验证丢包恢复和关键帧行为。
4. ARM64 厂家库、目标机器人 SSH/权限、实体急停、悬空测试和 30 分钟断线/重连耐久测试必须在机器狗上完成。

