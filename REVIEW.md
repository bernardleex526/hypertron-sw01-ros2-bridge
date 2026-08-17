# 实现与安全评审（直连驱动）

评审对象为 Hypertron SW01 ROS2 直连驱动：PC 上的 `hypertron_driver` 节点直连
官方 ASTRALL SDK 1.0.7 访问机器人，无 SSH、无 HTBR、无机器人端 agent。
本报告记录已完成的任务与遗留项。

## 任务状态

### Task 1–7：已完成并本地验证

| 任务 | 状态 |
|---|---|
| Task 1 构建契约（CMake/package.xml/合同 pytest） | 完成 |
| Task 2 SDK 抽象与网络预检（`astrall_sdk`/`direct_astrall_sdk`/`network_preflight`） | 完成 |
| Task 3 安全控制器改造（`RobotController`：driver-ready 门、断连失效、急停锁存） | 完成 |
| Task 4 直连运行时（`direct_driver_runtime`：worker、心跳、断线重连、模式/运动循环） | 完成 |
| Task 5 ROS 驱动节点（`driver_node.cpp` / `driver_main.cpp`，`hypertron_driver_node` 可执行） | 完成 |
| Task 6 配置/launch/文档（`config/driver_config.yaml`、`launch/driver.launch.py`、README/SW01_MANUAL_NOTES 更新、旧 SSH/agent 代码退役） | 完成 |
| Task 7 工作区集成（ros2_ws symlink + colcon build + launch 存活冒烟 + 旧 `sw01_ros2_driver` COLCON_IGNORE） | 完成 |
| **分期 3–4 LiDAR 点云/里程计接入**（`subscribe_lidar` discard 订阅、UDP 6100/6101 收流、`/points` 与 `/odom_lidar` 发布、`lidar_stream` 库集成、节点 5 用例 + 运行时 2 用例） | 完成 |
| **SLAM/Nav2 骨架**（launch/config，待标定） | 完成（骨架） |

验证矩阵：

- **纯 CMake**（`-DBUILD_ROS2_BRIDGE=OFF`）：5 项 CTest
  （`thread_safe_queue`、`robot_controller`、`network_preflight`、
  `direct_driver_runtime`（45 个用例）、`lidar_stream`）。退役的
  `protocol_handler`/HTBR 编解码及测试已删除。
- **ROS**（`-DASTRALL_SDK_ROOT=...`）：9 项 CTest
  （上述 5 项 + `package_contract` pytest 13 项、`astrall_sdk`、
  `direct_astrall_sdk`、`driver_node` 15 个用例）。
- **pytest 合同**：`test/test_package_contract.py` 13 项。
- **colcon**：`colcon build --packages-select hypertron_ros2_bridge` 通过；
  `ASTRALL_SDK_ROOT` 默认本机 SDK 路径（可覆盖）。
- **launch 存活冒烟**：无网络（eno1 无载波）下 `ros2 launch
  hypertron_ros2_bridge driver.launch.py` 运行 8s 节点存活、持续 preflight
  重试、无连接、零致动，被 timeout 正常终止（exit 124）；日志无 LiDAR fatal
  （无网络时 preflight failed 正常）。
- 安装产物：`readelf` 确认 DT_NEEDED `libASTRALL_SDK.so.1` 与 SDK RUNPATH；
  `ros2 pkg executables` 仅列出 `hypertron_driver_node`。

## 遗留项

### 实机验证 — 未进行
代码级测试不能替代实机运动安全认证。需在目标 PC 与 SW01 上完成：架空或稳定
支撑下的联调、实体急停可用、遥控器抢权、断线/重连耐久（建议至少 30 分钟），
以及 ROS 图命令（`ros2 topic list` 等）在目标环境验证。eno1 当前无载波
（网线未接/机器人未开），全部实机步骤待做。

### 本轮修复后的重新验证 — 未进行
本轮修复了 frames 参数失效、模式切换 promise 悬挂/覆盖、robot_state 竞态、
模式切换前零速、CMake 环境变量、无效 scan remap 等问题，并删除了退役 HTBR
代码。**尚未在 ROS 2 + ASTRALL SDK 环境重新构建和运行全部测试**；合并后必须
先完成 README 第 6 节验证。

### ThreadSanitizer — 建议抽查
建议用 `setarch -R`（关闭 ASLR）重跑 TSAN 抽查并发路径；上一轮曾观察到 WSL
TSAN 的 ASLR 兼容问题。

### pointcloud_to_laserscan 未安装

`config/laserscan_converter.yaml` 与 `launch/mapping.launch.py` 引用了
`ros-humble-pointcloud-to-laserscan`，但本机**未安装**该包。用户需
`sudo apt install ros-humble-pointcloud-to-laserscan`；在此之前
`mapping.launch.py` 完整启动会报 executable not found（预期，不做容错）。
`package.xml` 已声明 `pointcloud_to_laserscan` 为 per-包 exec_depend（ROS 2
中由用户显式 apt 安装）。

### LiDAR 比例标定 — 未进行
`/points`（`lidar.point_position_scale=1e-3`）与 `/odom_lidar`
（`odom_position_scale=1e-6`、`odom_quaternion_scale=1.0`）用的是手册默认比例，
**未实机抓包标定**。需用实机抓包（SOF `0xAA55` / TAIL `0xFF00`、packed/aligned
两种布局）确认坐标比例与四元数方向后再作数值真值使用；建图需先确认点云几何，
再接入 `pointcloud_to_laserscan` 等转换。提供点云/里程计旁路流不构成几何或里程
真值保证。

## 提交记录

直连驱动工作与后续修复提交至主分支；旧 SSH/agent 架构历史保存在远端
`archive/legacy-ssh-agent` 分支备查。

## 说明

- 本报告不构成实机运动安全认证。所有实机步骤均应由具备厂家授权和现场安全
  责任的操作者执行。
- `odometry.scale_verified=false` 且默认 `odom.publish_tf=false`，标定前不得把
  里程计数值当作物理真值。
