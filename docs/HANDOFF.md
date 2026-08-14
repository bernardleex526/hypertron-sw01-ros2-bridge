# 交接文档：SW01 直连 ROS 2 驱动（feature/direct-astrall-driver）

> 面向接手的 agent。本项目目标：把 Hypertron SW01 机器人接入 ROS 2 Humble，使用官方
> ASTRALL SDK 1.0.7 x86_64 库在 PC 本地经 UDP 直连机器人，**不再使用 SSH/HTBR/机器人端
> agent**。本文件说明架构、已完成项、未完成项、当前风险与验证命令。

---

## 1. 背景与关键事实

### 硬件/协议（官方文档确认）
- 机器人（服务端）：`10.18.0.100`，UDP 端口 `3600`
- PC（SDK 宿主）必须配置：`10.18.0.200`
- 官方 SDK：`/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++`
  - x86_64 库：`C++/lib/linux/x86_64/libASTRALL_SDK.so`（本驱动使用）
  - 头文件：`C++/include/interface.h`
  - **库内硬编码目标地址 10.18.0.100:3600，无运行时配置项**
- 手册（只读参考）：
  - `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/Hypertron-SW01 软件开发手册-中文版.pdf`
  - `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/璇玑屹励系列软件开发手册-中文版.pdf`
  - `/home/lee/zkyd/xuanji/Hypertron-SW01-电气硬件接口文档-20260710.pdf`
- SDK 能力：速度控制 `AstrallMove(vx,vy,vyaw)`（各 ±1）；模式 `DAMPING/FIXEDSTAND/FIXEDDOWN/MOVE/AUTOCHARGE/EXITCHARGE/GET_RIGHT`（稳定状态 B101/B102/B103/B104/B107 起始/B10B 过程/B1FF）；控制权 `ASTRALL_AUTH_SDK(1)` 申请、`ASTRALL_AUTH_JOYSTICK(2)` 交还遥控器（SDK 无独立“释放”枚举）；订阅 IMU/SPORT（1~250Hz）；**无关节接口**（`/joint_commands` 拒绝、不发布 `/joint_states`）；相机 H264→6000、点云 6100、里程计 6101 属后续范围。
- 心跳：推荐 10Hz；SDK 500ms 未收心跳清指令、再 500ms 判定断连。

### 主机网络（已配置，与仓库无关）
- mihomo（clashctl）：`/home/lee/clashctl/resources/mixin.yaml` 与 `runtime.yaml` 的
  `tun.route-exclude-address` 已加入 `10.18.0.0/24`，避免机器人流量进 VPN；已重启生效。
- NetworkManager 连接「有线连接 1」地址已改为 `192.168.123.99/24, 192.168.1.5/24, 10.18.0.200/24`。
- 最后检查时 `eno1` 无载波（网线未接/机器人未开）——**实机验证全部待做**。

### 仓库与工作区
- 主仓库（旧 SSH/HTBR 架构，main 分支，勿动）：
  `/home/lee/hypertron-sw01-ros2-bridge`
- **工作区（所有新工作都在这里）**：
  `/home/lee/hypertron-sw01-ros2-bridge/.worktrees/direct-astrall-driver`
  分支 `feature/direct-astrall-driver`；`.worktrees/` 已加入 `.git/info/exclude`。
- ROS 工作区：`/home/lee/ros2_ws`（colcon，含旧包 `sw01_ros2_driver` 与 `odin_ros_driver`；
  后续 Task 7 需要把本仓库 symlink 进去并给旧 `sw01_ros2_driver` 加 `COLCON_IGNORE`）。
- 设计文档：`docs/superpowers/specs/2026-08-13-direct-astrall-ros2-driver-design.md`
- 实施计划：`docs/superpowers/plans/2026-08-13-direct-astrall-ros2-driver.md`
- **至今未做任何 git commit**（按用户要求）。

---

## 2. 目标架构

```
ROS 2 应用
  <-> HypertronDriverNode          （Task 5 未实现）
  <-> DirectDriverRuntime          （Task 4 已完成，见下）
  <-> DirectAstrallSdk             （Task 2 完成）
  <-> x86_64 libASTRALL_SDK.so     （仅 direct_astrall_sdk.cpp 可 include interface.h）
  <-> UDP 10.18.0.100:3600
  <-> SW01
```

安全核心原则：启动非致动（不申请控制权/不切模式/不发速度）；重连绝不恢复授权/模式/速度；
断连即清控制器状态；软件急停锁存不随断连清除、解除不重放；关闭时仅在自己持有授权时发零速+阻尼。

---

## 3. 已完成（含验证证据）

### Task 1 ✅ 构建契约
- `CMakeLists.txt`：`ASTRALL_SDK_ROOT` cache 变量 + `BUILD_ROS2_BRIDGE` 作用域内 SDK
  查找（`include/interface.h` + `lib/linux/x86_64/libASTRALL_SDK.so`，缺失报
  "ASTRALL SDK x86_64 header/library not found"）；`hypertron_driver_node` 目标（`--no-as-needed`
  保证 DT_NEEDED 含 `libASTRALL_SDK.so.1`，build/install RPATH 指向 SDK）；生产构建已移除
  libssh/agent/camera；旧 SSH/agent 源仅保留在 `BUILD_TESTING` 供旧测试编译（Task 6 删除）。
- `package.xml`：描述改为 direct UDP ROS 2 driver，移除 `libssh-dev`，新增 `ament_cmake_pytest`。
- `test/test_package_contract.py`：13 个合同测试（SDK 查找作用域/路径/链接产物 readelf 断言、
  无效 root 失败、复用 build 目录换 root 失败、vendor include 唯一性、license 等），已注册进 CTest。

### Task 2 ✅ SDK 抽象与网络预检
- `include/hypertron_ros2_bridge/astrall_sdk.hpp`：项目自有类型（Result/SdkSnapshot/Velocity/
  ImuSample/SportSample/SubscriptionFrequency/SdkCallbacks）+ `IAstrallSdk`（init/deinit/
  heartbeat/request_authority/subscribe_imu/subscribe_sport/move/set_mode/snapshot）+ `DirectAstrallSdk`。
- `include/hypertron_ros2_bridge/astrall_vendor_api.hpp`：厂商函数表 seam（`AstrallVendorApi` +
  `make_real_astrall_vendor_api()`），供测试注入 fake backend。
- `src/direct_astrall_sdk.cpp`：**唯一**生产 TU include 厂商 `interface.h`。状态机
  Uninitialized/Initializing/Initialized/ShuttingDown；callback gate（in_flight+cv）；所有厂商
  入口（含 snapshot getter）由 lifecycle mutex 串行化；同步 init/subscribe 内回调可达且失败回滚；
  短 buffer 校验后复制；deinit 关 gate→vendor deinit→等 in_flight==0→原子发布 Uninitialized；
  **teardown 只在非回调线程执行**（回调线程 deinit 只置 deferred，由后续 deinit/checkpoint 消费）；
  重入拒绝（snapshot 走缓存，其余返回 Running "reentrant"，deinit 延迟）。**契约：适配器不得在厂商
  回调线程析构；运行时先 deinit 再由非回调线程析构。**
- `include/hypertron_ros2_bridge/network_preflight.hpp` + `src/network_preflight.cpp`：
  只读 `ip addr`/`ip route get` 事实采集（fork/execvp 无 shell、O_CLOEXEC、O_NONBLOCK 排水、
  5000ms 有界超时 SIGKILL+回收、EINTR/退出码完整处理、精确 CIDR 匹配）。
- 测试：`test_astrall_sdk`(8)、`test_direct_astrall_sdk`(27)、`test_network_preflight`(32)。

### Task 3 ✅ 安全控制器改造
- `RobotController`：`set_negotiated_ready`→`set_driver_ready`（旧名保留 deprecated 转发给旧
  agent 编译用，Task 6 删除）；新增线程安全 `mode_transition_pending()`；新增
  `invalidate_connection()`（清 ready/pending/速度，**保留 estop 锁存**）。测试 `test_robot_controller`(10)。

### Task 4 ⚠️ 直连运行时（已实现+多轮并发审查，但最后一轮修复未验证）
- `include/hypertron_ros2_bridge/direct_driver_runtime.hpp` + `src/direct_driver_runtime.cpp`：
  - 注入 `IAstrallSdk`/`INetworkPreflight`/`IMonotonicClock`/`RuntimeConfig`/`RuntimeObserver`。
  - worker 循环：preflight→init→订阅→连接循环（heartbeat 100ms 参数化、state_poll 500ms、
    motion_refresh 20ms、模式 deadline 严格结算）→失败 invalidate_session（controller
    `update_robot_state({})` + `invalidate_connection`）→deinit→有界退避→重连。
  - 启动非致动；`set_driver_ready` 只在**本连接首个 sdk_linked snapshot** 时武装。
  - `submit_velocity` 全门禁（ready/link/authority/estop/error/MOVE/无 pending）；worker tick
    发 latest-value，deadman 100ms 归零。
  - `request_authority(true/false)`（false=交还遥控器）、`request_mode`（串行+稳定状态确认+超时）、
    `trigger_estop`（锁存优先、清 pending、零速+阻尼、不重放）——断连立即失败，future 回填。
  - `stop()`：observer 回调线程内**先判上下文再取 stop_mutex_**（三方死锁防护），不 join 直接返回；
    普通线程取 `stop_mutex_` join；`start()` 也取 `stop_mutex_` 串行化 worker_ 访问。
  - `link_drop_latched_` 锁存断连事件（init 内 false→true 也判死会话），纳入 request/estop/stop
    致动全部准入。
  - CMake 生产静态库 `hypertron_direct_runtime`（纯，无 vendor/ROS 依赖），测试链接该库。
- 测试 `test_direct_driver_runtime`：**截至最后一轮未验证前为 37 个**，本轮新加 2 个
  （`QueuedCommandNeverExecutesAfterLinkDrop`、`ConcurrentStatusCallbacksCannotResurrectAuthority`）。

### 主机侧全量验证命令（历史证据）
```bash
cd /home/lee/hypertron-sw01-ros2-bridge/.worktrees/direct-astrall-driver
# 纯 CMake（无 ROS）
cmake -S . -B /tmp/htbr-pure -DBUILD_ROS2_BRIDGE=OFF -DBUILD_TESTING=ON
cmake --build /tmp/htbr-pure -j && ctest --test-dir /tmp/htbr-pure --output-on-failure   # 8/8（最后一轮修复前）
# ROS
source /opt/ros/humble/setup.bash
cmake -S . -B /tmp/htbr-ros -DASTRALL_SDK_ROOT=/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++ -DBUILD_TESTING=ON
cmake --build /tmp/htbr-ros -j && ctest --test-dir /tmp/htbr-ros --output-on-failure     # 11/11（含 pytest 13 项）
python3 -m pytest test/test_package_contract.py -q
```

---

## 4. ⚠️ 当前未完成/风险（接手第一优先级）

### A. Task 4 最后一轮修复「已改代码但未跑通验证」
上一轮 oracle 复核提出 3 项，我已直接修改 `src/direct_driver_runtime.cpp`：
1. `handle_status()` 的 controller 更新移入 `wake_mutex_` 临界区（消除回调互写复活授权）；
2. 连接循环在 wait 之后、执行命令之前复查 `link_drop_latched_/stop_requested_`（drop 后已入队
   命令不得执行）；
3. `trigger_estop(true)` 初始准入加 `link_drop_latched_`（latch 时立即失败）。

同时新增/修改测试：
- `test/test_direct_driver_runtime.cpp` 新增 `QueuedCommandNeverExecutesAfterLinkDrop` 与
  `ConcurrentStatusCallbacksCannotResurrectAuthority`；`LatchedLinkDropRejectsCommandsAndStopActuation`
  的 estop 断言改为 `wait_for(0s)` 立即就绪。
- **⚠️ 风险**：`QueuedCommandNeverExecutesAfterLinkDrop` 的写法可能自挂——它在 worker 停泊于
  snapshot 钩子内时等待 "connection lost" 事件（该事件要 worker 退出会话后才发出），若断言先失败
  会提前返回、析构 join 停泊中的 worker → 死锁。**接手后先修这个测试的释放顺序（先释放钩子再等待
  事件），再跑全量验证。**
- 当时验证时 shell 会话卡死（测试二进制挂起 + 终端会话阻塞），未完成构建/测试确认。**建议新开
  终端/新会话执行上面的验证命令。**

### B. Task 5 ❌ ROS 节点（未开始）
在 DirectDriverRuntime 之上实现 `HypertronDriverNode` + `src/driver_main.cpp`（driver_main.cpp
目前只是惰性 skeleton）：
- 订阅 `/cmd_vel`（Twist）、`/robot_mode`（String）、`/joint_commands`（JointState，一律拒绝）；
- 发布 `/imu/data`（SensorDataQoS）、`/odom`（诊断，`odometry.scale_verified=false` 且默认不发
  odom→base TF）、`/robot_state`（强类型 `msg/RobotState.msg`；旧字段 `ssh_connected/agent_connected`
  兼容保留恒 false，`sdk_linked` 为真实 UDP 链路）；`/joint_states` 不发布。
- 服务 `/emergency_stop`（SetBool）、`/control_authority`（SetBool：true 申请 SDK 权，false 交还
  遥控器）。
- 参数化 topic/frame 名、QoS、协方差、四元数顺序（默认 xyzw）等；`RuntimeObserver` 实现把
  on_state/on_imu/on_sport/on_status/on_event 转成 ROS 发布。注意 observer 回调来自 vendor 线程，
  只能调用线程安全的 ROS 操作或经节点 executor 转发。

### C. Task 6 ❌ 配置/启动/文档/旧代码退役（未开始）
- 新建 `config/driver_config.yaml`（仅有效参数：心跳 100ms、deadman 100ms、模式超时、订阅频率、
  帧名、协方差、odometry/TF 默认关）与 `launch/driver.launch.py`；替换旧 `config/bridge_config.yaml`。
- 重写 `README.md`（构建命令、只读联调、手动申请控制权、实机安全 SOP）、更新 `SW01_MANUAL_NOTES.md`、
  `REVIEW.md`；删除 `src/bridge_node.cpp`、`src/main.cpp`、`src/ssh_tunnel.cpp`、`src/agent_main.cpp`、
  `src/agent_runtime.cpp`、`src/posix_byte_stream.cpp`、`src/astrall_sdk_adapter.cpp` 及对应头文件、
  HTBR 文档、旧 SSH/agent 测试与 CMake 相关 target；删除 `RobotController::set_negotiated_ready`
  的 deprecated 别名。**先保证无 target 引用再删。**
- `msg/RobotState.msg` 字段顺序保持不动（兼容）。

### D. Task 7 ❌ 工作区集成与主机侧验证（未开始）
- symlink `/home/lee/ros2_ws/src/hypertron_ros2_bridge` → worktree 路径；`colcon build`；
- 旧 `/home/lee/ros2_ws/src/sw01_ros2_driver` 在替换验证通过后加 `COLCON_IGNORE`（避免双进程抢 UDP 3600）；
- `timeout 10s ros2 launch hypertron_ros2_bridge driver.launch.py`（无网络时节点存活并重试、零致动）；
- `ldd`/`readelf` 验证 SDK NEEDED/RUNPATH；`ros2 pkg executables` 只列出直连驱动。

### E. 实机验证 ❌（完全未做，必须分阶段）
eno1 无载波状态未知，需要：确认路由 `ip route get 10.18.0.100` 走 eno1、src=10.18.0.200；机器人
架空+实体急停可达+遥控器在场；先只读联调（连接/设备/系统/电池/IMU/轮速），再手动申请控制权，
再 stand→move→低速→明确零速→ROS deadman→软件急停→物理急停→断线重连→遥控器抢权。测试/构建
通过不等于实机安全。

---

## 5. 约定与注意
- **不提交 git**（用户要求，除非另行指示）。
- 厂商 SDK 是闭源二进制：只引用路径，不拷贝进仓库。
- `interface.h` 的 include 全仓库唯一（仅 `src/direct_astrall_sdk.cpp`），有 pytest 防回归断言。
- 子代理实现时已知坑：fixer/oracle 会话偶发返回空结果（上下文读了文件但零写入）——接手时先核对
  文件实际 diff 再采信报告；小修可直接由编排者动手（本会话末尾的 3+2 处修复即直接修改）。
- 测试风格约定：并发/时序测试用 cv/事件等待与 fake 钩子（`on_move_enter/on_snapshot_enter/
  on_init_enter/set_snapshot_blocked/emit_status`），禁止真实 sleep 断言；先释放停泊钩子再 ASSERT。
- TSAN 可用 `setarch -R` 规避 gcc11 libtsan 与本机内核 ASLR 不兼容（`-fsanitize=thread -O1 -g`）。
