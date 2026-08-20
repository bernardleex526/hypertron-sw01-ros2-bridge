# SW01 ROS2 桥接 · 标定/建图/导航交接文档

> 本文档面向接手同事，记录当前机器上可用的文件夹、已完成的修复/标定、常用命令和排查方法。
> 所有实机运动步骤必须满足安全前提：机器人稳定支撑/架空、实体急停可达、遥控器在场。

---

## 1. 关键路径

| 项目 | 路径 |
|---|---|
| ROS2 桥接仓库 | `/home/lee/hypertron-sw01-ros2-bridge` |
| 已构建安装目录 | `/home/lee/hypertron-sw01-ros2-bridge/install/setup.bash` |
| ASTRALL SDK | `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++` |
| SDK 动态库 | `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++/lib/linux/x86_64/libASTRALL_SDK.so` |
| 官方手册 PDF | `/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/Hypertron-SW01 软件开发手册-中文版.pdf` |
| 标定分析脚本 | `/home/lee/hypertron-sw01-ros2-bridge/tools/calibration_analyze.py` |
| QoS 转发节点 | `/home/lee/hypertron-sw01-ros2-bridge/tools/qos_relay.py` |
| rviz2 点云配置 | `/home/lee/hypertron-sw01-ros2-bridge/config/points_view.rviz` |
| 实机标定 Runbook | `/home/lee/hypertron-sw01-ros2-bridge/docs/REAL_MACHINE_CALIBRATION.md` |

---

## 2. 架构简介

- PC 直连机器人，无 SSH、无机器人端 agent。
- 控制链路：ASTRALL SDK，UDP `3600`，目标 `10.18.0.100:3600`（SDK 内硬编码）。
- LiDAR 点云：UDP `6100`。
- LiDAR 里程计：UDP `6101`。
- ROS 节点：
  - `hypertron_driver_node`：SDK 控制 + 发布 `/points`、`/odom_lidar`、`/imu/data`、`/robot_state`；
    UDP 6100 点云使用 `lidar_points`，UDP 6101 里程计子帧使用 `lidar`，两者不可混用。
  - `qos_relay`：把 BEST_EFFORT 传感器话题转发为 RELIABLE 话题，解决 Humble rviz2 / pointcloud_to_laserscan 的 QoS 不兼容。
  - `pointcloud_to_laserscan_node`：`/points_relay` → `/scan`。
  - `async_slam_toolbox_node`：`/scan` + TF → `/map`。

---

## 3. 已完成的修复

### 3.1 点云组帧修复

- 实机抓包确认：UDP 6100 的 `index` 不是包序号，而是**本包在帧内的起始点偏移**（0、50、100...）。
- 原代码按包序号 `0,1,2,...` 组帧，导致 `/points` 永远不发布。
- 已修改 `src/lidar_stream.cpp` / `include/.../lidar_stream.hpp`：
  - 组帧改为按点区间精确覆盖 `[0,total)`，无重叠、无空洞才发布。
  - `/points` 实测约 10Hz。

### 3.2 QoS 转发

- 驱动 `/points`、`/odom_lidar`、`/imu/data` 为 BEST_EFFORT。
- Humble rviz2 与 pointcloud_to_laserscan 默认使用 RELIABLE，无法直接接收。
- `tools/qos_relay.py` 转发：
  - `/points` → `/points_relay`
  - `/odom_lidar` → `/odom_lidar_relay`
  - `/imu/data` → `/imu_relay`
- `mapping.launch.py` 已内置启动该 relay。

### 3.3 TF 树与点云轴向修正

- 驱动发布 `odom -> lidar`。
- 实机在机器人正前方约 1m 放置承重柱，UDP 6100 原始点云主方向为 `-137°`；
  将原始点旋转 `+137°` 后，柱子中心约为 `(x=1.022m, y=0.006m)`。
- `/points.header.frame_id` 使用独立的 `lidar_points`，`mapping.launch.py` 发布
  `lidar -> lidar_points`：平移 `(0,0,0)`、yaw `+137°`（`2.391101075rad`）。
- 静态 TF 必须为 `lidar -> base_link`，才能组成 `odom -> lidar -> base_link`。
- `mapping.launch.py` 已修正：
  - `lidar -> lidar_points`：`(0, 0, 0, yaw=+137°)`
  - `lidar -> base_link`：`(-0.3732, 0, -0.080)`
  - `base_link -> imu_link`：`(-0.00115, 0, 0.0595)`

### 3.4 参数标定结果

根据实机抓包和物理测量：

| 参数 | 值 | 依据 |
|---|---|---|
| `lidar.point_position_scale` | `0.0000191` | 1m 物体原始距离约 52244 |
| `lidar.frame_id` | `lidar_points` | 与 6101 的 body-aligned `lidar` frame 分离 |
| `lidar_points_yaw` | `2.391101075` | 正前方柱子原始方位 `-137°` |
| `lidar.odom_position_scale` | `0.0000009474` | 前进 1m 原始 Δx = -1,055,486 |
| `lidar.odom_quaternion_order` | `xyzw` | 原地旋转 90°，z/w 分量变化正确 |
| `odometry.scale_verified` | `true` | 允许发布 odom TF |

已写入：

```text
config/driver_config.yaml
```

---

## 4. 当前状态

- 2026-08-18 重构建后实机复测：`/points_relay` 与 `/scan` 均为约 `10.28Hz`，
  `/points_relay.header.frame_id=lidar_points`。
- `odom -> base_link` TF 正常。
- 正前方柱子经 `lidar -> lidar_points` 修正后位于雷达 body-aligned frame 的
  `x≈1.025m, y≈0.005m`、方位约 `0.27°`、水平距离中位数约 `1.027m`，
  距离与水平方向初验通过。
- 同一帧点云自身的 Z 范围约 `-0.043～0.695m`，未见应位于
  `z≈-0.417m` 的地面回波；`odom->lidar.z≈0.417m`，因此障碍物回波在 RViz
  中高于 `odom` 的 `z=0` Grid 是当前数据的正常结果。未实测雷达光学中心
  离地高度前，不得为了“贴 Grid”而整体下移点云。
- `mapping.launch.py` 已实测可启动，`/scan` 正常输出。
- `mapping.launch.py use_rviz:=true` 会自动加载 `config/points_view.rviz`；该配置
  保持 Fixed Frame=`odom`，并让视图跟随 `lidar`，无需再靠 F 键寻找点云。

---

## 5. 常用命令

### 5.1 仅启动驱动（测试控制/点云）

```bash
pkill -f hypertron_driver_node || true

source /opt/ros/humble/setup.bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

ros2 launch hypertron_ros2_bridge driver.launch.py
```

### 5.2 启动 QoS relay（如果需要单独转发）

```bash
source /opt/ros/humble/setup.bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

python3 /home/lee/hypertron-sw01-ros2-bridge/tools/qos_relay.py
```

### 5.3 启动建图

**不要同时运行 driver.launch.py。**

```bash
pkill -f hypertron_driver_node || true
pkill -f mapping.launch.py || true

source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

ros2 launch hypertron_ros2_bridge mapping.launch.py \
  enable_tf_skeleton:=true \
  enable_odom_tf:=true \
  use_rviz:=true
```

用遥控器控制机器狗走动建图。

### 5.4 rviz2 看点云

```bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash
ros2 run rviz2 rviz2 -d /home/lee/hypertron-sw01-ros2-bridge/config/points_view.rviz
```

或在 rviz2 中：

- Fixed Frame = `odom`
- 添加 PointCloud2：`/points_relay`
- 添加 Map：`/map`

### 5.5 保存地图

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### 5.6 导航

```bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

ros2 launch hypertron_ros2_bridge navigation.launch.py \
  map:=/home/lee/map.yaml \
  use_amcl:=true
```

或在线 SLAM 配合导航：

```bash
ros2 launch hypertron_ros2_bridge navigation.launch.py use_amcl:=false
```

---

## 6. 标定数据复现

如果需要重新标定或验证，抓包命令：

```bash
# 直线前进 1m
sudo timeout 30 tcpdump -i eno1 -w /tmp/odom_forward.pcap udp port 6101

# 原地旋转 90°
sudo timeout 30 tcpdump -i eno1 -w /tmp/odom_rotate.pcap udp port 6101

# 已知距离 1m 物体
sudo timeout 10 tcpdump -i eno1 -w /tmp/lidar_known.pcap udp port 6100
```

分析：

```bash
python3 /home/lee/hypertron-sw01-ros2-bridge/tools/calibration_analyze.py \
  --odom /tmp/odom_forward.pcap

python3 /home/lee/hypertron-sw01-ros2-bridge/tools/calibration_analyze.py \
  --odom /tmp/odom_rotate.pcap

python3 /home/lee/hypertron-sw01-ros2-bridge/tools/calibration_analyze.py \
  --cloud /tmp/lidar_known.pcap
```

---

## 7. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `runtime is disconnected` | 有两个 `hypertron_driver_node` 同时在跑 | `pkill -f hypertron_driver_node`，只保留一个 |
| `/points` 无数据 | 旧代码组帧语义错误 / 旧安装 | 使用仓库 install 并重新 `colcon build` |
| rviz2 无点云 | QoS 不兼容 / Fixed Frame 错误 | 使用 `/points_relay`，Fixed Frame 设 `odom` |
| `/scan` 无数据 | pointcloud_to_laserscan 收不到 `/points` | 确保 `mapping.launch.py` 内置 relay 已启动，输入是 `/points_relay` |
| TF 报 two trees / base_link 不存在 | 静态 TF 方向错误 | 静态 TF 应为 `lidar -> base_link` |
| `slam_toolbox` 报 min laser range 警告 | `min_laser_range=0.0` 小于实际 0.1m | 可忽略，或改 `config/slam_toolbox_online_async.yaml` 为 `0.1` |
| 地图尺寸不对 | 点云/里程计比例不准 | 重新标定 `point_position_scale` / `odom_position_scale` |
| 点云整体高于 Grid | Grid 是 `odom` 的 `z=0` 平面，当前点云没有地面回波且雷达高度约 0.417m | 先实测地面到雷达光学中心；不要为贴 Grid 直接改 TF 高度 |

---

## 8. 尚未完成/后续注意

- `lidar -> base_link` 平移目前采用官方手册数值，尚未用实测校准；若地图方向/位置有偏差，需要重新测量并修改 `mapping.launch.py`。
- `lidar -> lidar_points` yaw 基于单个正前方柱子标定；正式验收前应再用左侧/右侧目标或小角度转向复核 Y 轴符号和角度。
- 点云比例基于约 1m 柱子测量（修正后中位距离约 1.022m），建议再测 2m/3m 复核线性度。
- 当前点云 Z 范围实测约 `-0.043～0.695m`，未见明显地面点；需实测地面到
  雷达光学中心高度，并与 `odom->lidar.z≈0.417m` 对照后再验收 Z 轴/地面高度。
- 里程计比例基于单次 1m 直线，建议多测几次取平均。
- 导航参数仍为保守骨架值，正式使用前需根据实际机器人尺寸和运动能力调整。
