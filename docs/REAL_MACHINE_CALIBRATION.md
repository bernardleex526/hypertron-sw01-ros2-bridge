# SW01 直连驱动 · 实机联调与标定 Runbook

> 本文件是实机操作清单。**代码级测试与本地构建通过 ≠ 实机安全**。
> 所有实机步骤必须满足：机器人架空或稳定支撑、实体急停随时可达、遥控器在场并
> 可随时抢权。任一安全前提不满足，不得进入运动相关步骤。

## 0. 阶段总览

| 阶段 | 内容 | 致动 | 前提 |
|---|---|---|---|
| A | 网络与只读联调 | 否 | 网线、机器人上电 |
| B | 抓包与协议标定（6100/6101） | 否（可含小范围被动机测） | A 通过 |
| C | 参数回填 + 复测 | 否 | B 完成 |
| D | 手动控制权 + 低速运动验证 | **是** | C 通过、安全三前提 |
| E | 建图（SLAM 骨架） | 是（低速） | D 通过 |
| F | 导航（Nav2 骨架） | 是（低速） | E 通过、地图验收 |

**未经 B/C 标定，禁止使用任何点云/里程计数据做定位或导航。**

## A. 网络与只读联调

1. 接网线，确认载波与路由（预期 eno1、src=10.18.0.200）：
   ```bash
   ip -brief addr show eno1
   ip route get 10.18.0.100
   ping -c 3 10.18.0.100
   ```
   代理（mihomo）须已排除 10.18.0.0/24（交接期已配置，若换环境重查）。
2. 启动驱动并只读观察（**不申请控制权、不切模式、不发速度**）：
   ```bash
   source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
   ros2 launch hypertron_ros2_bridge driver.launch.py
   ros2 topic echo /robot_state --once
   ```
   验收：`sdk_linked: true`、`control_authority: false`、`emergency_stop: false`、
   电池/系统字段合理；`ros2 topic hz /imu/data` 与订阅频率一致（默认 50Hz）；
   `/points` 与 `/odom_lidar` 有数据（说明 LIDAR 开流成功——这是本阶段重点）。
   若 `/points` 无数据：检查节点日志中 lidar 相关 warning，以及 `lidar.source_ip`
   与 `lidar.bind_ip` 参数。

## B. 抓包与协议标定（决定一切上层用法）

**抓包**（需 sudo，机器人上电、驱动已启动开流）：
```bash
sudo tcpdump -i eno1 -w /tmp/lidar6100.pcap udp port 6100 -c 2000
sudo tcpdump -i eno1 -w /tmp/odom6101.pcap udp port 6101 -c 500
```
用 `tcpdump -r ... -X | head` 与脚本统计，逐项确认下表并**记录到本文件末尾的
标定记录表**：

| # | 待确认项 | 方法 | 预期（未证实） |
|---|---|---|---|
| B1 | 6101 布局 | 统计 datagram 长度分布 | 仅 68（packed）或 80（aligned） |
| B2 | 6100 布局 | 长度 = 20+28N+2 或 28+28N+2（packed/aligned，动态尾）或固定 50 点尾 1422/1430 | 与解析器四布局之一吻合；同时确认 offset 10 是“帧总点数”还是“帧序号”，以及单点 28B 中后 16B 是否为 4 个 RGBA 分量 |
| B3 | 头尾标记 | 首两字节 0xAA55、尾 0xFF00 | 一致 |
| B4 | 比例：点坐标 | 已知距离物体（如 1m 标尺）在点云中的坐标差 | int32 × 1e-3 ≈ 米 |
| B5 | 比例：odom 位置 | 推车直线 1m，Δ(x,y,z) | int64 × 1e-6 ≈ 米 |
| B6 | 比例：odom 姿态 | 原地旋转 90°，四元数 ≈ (0,0,sin45°,cos45°) | 注意：解析后统一归一化，scale 参数会被约掉；真正要标定的是分量顺序 |
| B7 | 四元数顺序 | B6 验证（w 是否在末位） | 当前代码固定按 qx,qy,qz,qw 读取，尚无顺序参数；若实机为 wxyz，代码需新增 order 配置 |
| B8 | 轴向 | 前进方向对应哪个坐标轴、z 向上 | 待确认 |
| B9 | index 基 | 抓包统计每帧 index 集合 | 0-based 或 1-based |
| B10 | 单/双雷达 | 每帧 total 点数与频率；点云是否含两雷达混合 | SW01 预期单雷达 |
| B11 | 6101 参考系 | 旋转中心近似位置（child_frame 是 lidar 还是 base） | 待确认 |
| B12 | 设备时间戳 | timestampNs 与接收时刻差值是否恒定（相对时钟）或近似 unix 秒级 | 待确认 |
| B13 | 丢包/乱序率 | 抓包期间统计缺包 | 重组器已容忍乱序，丢包率需 <1% 量级 |

**B4/B5/B6 涉及推动机器人——必须安全三前提在场；移动前确认实体急停有效。**

## C. 参数回填与复测

按 B 的结果更新 `config/driver_config.yaml`：
- `lidar.point_position_scale` / `lidar.odom_position_scale` /
  `lidar.odom_quaternion_scale`（实测值）；
- `lidar.frame_id`（与 URDF/外参一致）、`odom_lidar.parent_frame/child_frame`；
- 若 6101 参考系与 lidar 外参已确认，可开启后续 TF（**本驱动当前版本不发布
  odom→base TF；见 E 阶段骨架说明**）。

复测：重启驱动，`ros2 topic echo /odom_lidar` 静止时位置漂移 ≤ 阈值
（记录静止 60s 漂移值），直线/旋转误差达到 D 阶段准入（建议：直线 ±2%、
旋转 ±2° 内再谈定位）。

## D. 手动控制权 + 低速运动验证（沿用 README 安全 SOP）

顺序：只读确认 → `ros2 service call /control_authority std_srvs/srv/SetBool
"{data: true}"` → `stand` → `move` → 低速小位移 → 明确零速 → ROS deadman
（停发 /cmd_vel 100ms）→ 软件急停 → 物理急停 → 断网线观察重连与零致动 →
遥控器抢权验证驱动不得循环抢回。

## E. 建图（SLAM 骨架）

1. 安装缺失依赖（需管理员）：`sudo apt install ros-humble-pointcloud-to-laserscan`
   （robot_localization 可选，后续里程计融合再用）。
2. 复核 `lidar→lidar_points` 轴向和 `lidar→base_link` 外参（B8/B10 的轴向
   + 结构尺寸）后启动：
   ```bash
   ros2 launch hypertron_ros2_bridge mapping.launch.py \
     enable_tf_skeleton:=true enable_odom_tf:=true
   ```
3. 低速推/遥操作覆盖环境建图，保存地图，静止漂移与地图一致性验收。
4. 当前 `lidar→lidar_points` yaw 使用正前方约 1m 柱子的初验值 `+137°`；
   `lidar→base_link` 平移使用手册值。正式导航前仍须复核侧向轴、2m/3m
   距离线性度和雷达光学中心离地高度。

## F. 导航（Nav2 骨架）

1. 加载 E 阶段保存的地图：
   ```bash
   ros2 launch hypertron_ros2_bridge navigation.launch.py map:=<绝对路径>/map.yaml
   ```
2. 低速导航验收：点到点、避障、断流时感知超时（Nav2 应停而不是用陈旧数据）。

## 标定记录表（实机填写）

| 项 | 实测值 | 结论/回填参数 | 日期 |
|---|---|---|---|
| B1 6101 布局 | | | |
| B2 6100 布局 | | | |
| B3 头尾标记 | | | |
| B4 点比例 | | `lidar.point_position_scale=` | |
| B5 odom 位置比例 | | `lidar.odom_position_scale=` | |
| B6/B7 odom 姿态/顺序 | | | |
| B8 轴向 | 正前方柱子在 UDP 6100 原始 XY 方位约 -137°；旋转 +137° 后实机复测 x≈1.025m、y≈0.005m、方位≈0.27° | `lidar.frame_id=lidar_points`；`lidar→lidar_points yaw=2.391101075rad`；Y 轴仍需侧向目标复核 | 2026-08-18 |
| B9 index 基 | | | |
| B10 雷达数 | | | |
| B11 6101 参考系 | | `odom_lidar.child_frame=` | |
| B12 设备时钟 | `/tmp/lidar_known.pcap` 与 `/tmp/odom_forward.pcap` 中6100/6101均为设备启动后的单调纳秒；两路换算到抓包主机时钟的首包offset仅差约1.4ms，非Unix时间 | 驱动以6101接收时刻估计设备时钟到ROS时钟offset，6100复用该映射；仍需重启后和长时间漂移验收 | 2026-08-19 |
| B13 丢包率 | | | |
