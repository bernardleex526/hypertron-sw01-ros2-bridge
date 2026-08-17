# 交接要点（当前 main）

> 旧的交接文档曾记录 Task 5/6/7 未开始等历史状态，已过时。本文件只保留当前
> 接手者需要知道的事实；完整 SOP 与未解决问题以根目录 README.md 为准。

## 架构现状

- PC 端 `hypertron_driver` 节点直连厂商 ASTRALL SDK 1.0.7（UDP 3600）。
- 无 SSH、无 HTBR、无机器人端 agent；退役的 `protocol_handler` 已删除。
- LiDAR 点云/里程计走 UDP 6100/6101 旁路，由 `lidar_stream` 解析和组帧。
- 安全原则：启动/重连非致动、断连清状态、急停锁存不重放、关闭时仅在持有
  授权时发零速+阻尼。

## 接手第一优先级

1. 在目标 ROS 2 Humble + ASTRALL SDK 机器上重新跑 README 第 6 节测试
   （本轮修改尚未构建验证）。
2. 按 README 第 9 节处理厂商协议字段/RGBA/LiDAR 频率/四元数顺序等未决项。
3. 实机 SOP 只能按 README 第 8 节阶段 A→F 顺序执行，禁止跳阶段。

## 关键文件

- `src/direct_astrall_sdk.cpp`：唯一允许 include 厂商 `interface.h` 的 TU。
- `src/direct_driver_runtime.cpp`：worker、心跳、重连、模式/运动状态机。
- `src/driver_node.cpp`：ROS 图接口与参数校验。
- `src/lidar_stream.cpp`：UDP 6100/6101 解析与有界组帧（字段语义未实机证实）。
- `docs/REAL_MACHINE_CALIBRATION.md`：13 项实机标定矩阵。

## 安全声明

本仓库不构成实机运动安全认证。任何致动测试必须满足：机器人架空/稳定支撑、
实体急停可达、遥控器在场。
