# SW01 + Odin1 点云标定、建图、定位与导航 SOP

> 文档状态：2026-08-18 实机阶段性版本
>
> 适用仓库：`/home/lee/hypertron-sw01-ros2-bridge`
>
> ROS 版本：ROS 2 Humble
>
> 机器人：Hypertron SW01 轮足式机器狗
>
> 空间模组：留形科技 Odin1
> 原则：代码构建通过、节点启动、话题存在均不等于实机建图/定位/导航验收通过。

## 1. 安全边界

任何涉及机器人运动的步骤都必须同时满足：

1. 实体急停可立即触达；
2. 遥控器在场且可以立即抢权；
3. 机器人周围清场，首次运动应架空或稳定支撑；
4. 低速开始，先验证零速、断流自动停止和急停；
5. 不得同时启动两套 `mapping.launch.py` 或两个 `hypertron_driver_node`；
6. 未完成本 SOP 第 7～10 节标定和验收前，不允许开启自主实机导航。

本 SOP 中点云、TF、地图检查均为只读操作，不需要申请机器人控制权。只有明确标注为“运动步骤”的命令才允许在安全前提满足后执行。

## 2. 当前架构与数据链

### 2.1 当前 PC 到 SW01 的以太网链路

```text
当前 PC
  │ Ethernet（预期 eno1，PC 10.18.0.200）
  ▼
SW01 / Daedalus（10.18.0.100）
  ├─ UDP 3600：ASTRALL SDK 控制、状态和订阅
  ├─ UDP 6100：AstrallPosCloudData 点云
  └─ UDP 6101：AstrallOdometryData 雷达里程计
```

### 2.2 机器人内部推测链路

```text
Odin1
  │ 机器人内部 USB 3.x
  ▼
RK3588 / 厂商集成程序
  │ 点云和里程计转发
  ▼
Daedalus / ASTRALL
  │ Ethernet UDP 6100/6101
  ▼
当前 PC 的 hypertron_driver_node
```

当前 PC 能通过以太网取得点云和里程计，不代表当前 PC 能直接访问 Odin1 的 USB 管理接口。数据访问和 Odin 模式管理是两条不同链路。

### 2.3 当前 ROS 2 建图链路

```text
UDP 6100
  → /points（PointCloud2，BEST_EFFORT，frame=lidar_points）
  → qos_relay
  → /points_relay（RELIABLE）
  → pointcloud_to_laserscan
  → /scan（frame=base_link）
  → slam_toolbox async mapping
  → /map + map→odom

UDP 6101
  → /odom_lidar
  → odom→lidar
  → lidar→base_link 静态外参
```

当前算法不是 FAST-LIO 系列：

- 建图：二维 `slam_toolbox`，基于二维 LaserScan 扫描匹配、位姿图、回环检测和 Ceres 优化；
- 保存地图后的定位：AMCL，当前配置为 `DifferentialMotionModel`；
- 全局规划：NavFn，`use_astar=false`；
- 局部控制：DWB；
- 当前 `max_vel_y=0`，按不可侧移平台使用。

该组合适合“平整地面、轮式模式、较小俯仰横滚、禁止横移”的阶段性验证，不适合直接覆盖台阶、坡道、腿式步态和明显机身升降。

## 3. 关键路径

```text
仓库：              /home/lee/hypertron-sw01-ros2-bridge
安装空间：          /home/lee/hypertron-sw01-ros2-bridge/install
驱动参数：          config/driver_config.yaml
点云转 LaserScan：  config/laserscan_converter.yaml
slam_toolbox 参数： config/slam_toolbox_online_async.yaml
SLAM调参说明：      docs/SLAM_TOOLBOX_PARAMETER_GUIDE.md
Nav2 参数：         config/nav2_params.yaml
建图 launch：       launch/mapping.launch.py
导航 launch：       launch/navigation.launch.py
实机标定手册：      docs/REAL_MACHINE_CALIBRATION.md
交接记录：          docs/HANDOFF_CALIBRATION_MAPPING.md
抓包分析工具：      tools/calibration_analyze.py
实时参照物工具：    tools/live_cloud_reference_check.py
Odin 官方驱动：     /home/lee/odin_ws/src/odin_ros_driver
SW01 官方手册：     /home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/Hypertron-SW01 软件开发手册-中文版.pdf
```

## 4. 截至 2026-08-18 的实机进度

### 4.1 已确认

1. PC 与 SW01 通过以太网通信正常；
2. ASTRALL SDK 1.0.7 能建立连接；
3. UDP 6100 点云和 UDP 6101 里程计均有数据；
4. UDP 6100 的 `index` 已确认是帧内起始点偏移，不是包序号；
5. `/points`、`/points_relay`、`/scan` 正常输出；
6. 单套实例下 `/points_relay` 和 `/scan` 均约 10.2 Hz；
7. 点云 frame 与里程计 frame 已拆分为 `lidar_points` 和 `lidar`；
8. 正前方柱子标定得到点云轴向修正 yaw `+137°`；
9. `lidar→base_link=(-0.3732, 0, -0.080)` 来自 SW01 手册的雷达相对质心位置；
10. UDP 6101 四元数顺序初验为 `xyzw`；
11. 里程计位置比例已回填，允许发布 `odom→lidar` TF；
12. `mapping.launch.py`、RViz2、`slam_toolbox` 可以启动并输出 `/map`；
13. 最近一次地图输出为 0.05 m 分辨率、38×22 栅格；
14. 最近安全状态：`control_authority=false`、`emergency_stop=false`、四轮速度为 0；
15. 2026-08-18 柱子实时重复检查三帧结果：

```text
target_frame=lidar
x≈1.027 m
y≈0.003 m
水平距离≈1.028 m
方位≈+0.17°
三帧结果基本一致
```

这证明正前方向和静态重复性良好，但由于柱子实际物理距离未知，不能据此宣布绝对距离完成标定。

### 4.2 当前参数

```text
lidar.point_position_scale = 0.0000191
lidar.odom_position_scale  = 0.0000009474
lidar.odom_quaternion_order = xyzw
lidar.frame_id = lidar_points
lidar_points_yaw = 2.391101075 rad（+137°）
odometry.scale_verified = true
```

### 4.3 尚未完成或尚未证实

1. 点云 1 m、2 m、3 m 多距离线性度；
2. 左右目标或机器人 ±10°/±30° 的角度和 Y 轴符号复核；
3. 平墙直线拟合、墙面角度和绝对距离复核；
4. 雷达光学中心实际离地高度；
5. 点云 Z 轴、地面回波和高度裁剪范围；
6. UDP 6101 动态直线、旋转、多次重复误差；
7. UDP 6101 的参考原点是否严格等同 Odin 或机器人厂家定义的雷达原点；
8. UDP 6100 对应 Odin `/odin1/cloud_raw`、`/odin1/cloud_slam` 或厂商二次处理点云中的哪一种；
9. UDP 6101 对应 Odin 普通里程计还是高频里程计；
10. Odin1 当前运行 `Odometry`、`SLAM` 还是 `Relocalization` 模式；
11. RK3588 登录方式、Odin USB 拓扑和厂商进程占用情况；
12. 已于2026-08-19修改6100/6101设备时间映射、同一里程计消息/TF时间戳以及两级消息过滤队列；正式建图前仍须实机确认不再持续出现 `Message Filter dropping ... queue is full`；
13. `min_laser_range=0.11` 已与实际 LaserScan `range_min=0.1` 协调，并避开浮点边界警告；
14. Nav2 的机器人 footprint/半径、膨胀半径、障碍高度、探测距离均为占位值；
15. 尚未完成地图几何误差、回环、AMCL 重定位和自主导航验收。

## 5. 每次开始工作前的环境检查

### 5.1 网络

```bash
ip -brief addr show eno1
ip route get 10.18.0.100
ping -c 3 10.18.0.100
ip neigh show dev eno1
```

预期路由源地址为 `10.18.0.200`，目标机器人为 `10.18.0.100`。

### 5.2 检查是否已有实例

```bash
pgrep -af 'ros2 launch hypertron_ros2_bridge mapping.launch.py'
pgrep -af 'hypertron_driver_node|qos_relay.py|async_slam_toolbox_node|pointcloud_to_laserscan_node|rviz2'
```

如果已有一套正常实例，不要再次启动。

ROS 图检查：

```bash
source /opt/ros/humble/setup.bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

ros2 node list
ros2 topic info /points_relay -v
```

验收：`/points_relay` 的 `Publisher count` 必须为 1。

### 5.3 安全状态

```bash
ros2 topic echo /robot_state --once
```

只读标定和建图准备阶段应保持：

```text
sdk_linked: true
control_authority: false
emergency_stop: false
wheel_speed: [0, 0, 0, 0]
```

## 6. 构建、启动、验证与停止

### 6.1 仅在源码修改后重新构建

确保没有正在使用安装空间的节点，再执行：

```bash
cd /home/lee/hypertron-sw01-ros2-bridge
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select hypertron_ros2_bridge
```

检查：

```bash
ctest --test-dir build/hypertron_ros2_bridge --output-on-failure
```

### 6.2 启动 mapping

```bash
cd /home/lee/hypertron-sw01-ros2-bridge
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_LOG_DIR=/tmp/hypertron_mapping_roslog \
ros2 launch hypertron_ros2_bridge mapping.launch.py \
  enable_tf_skeleton:=true \
  enable_odom_tf:=true \
  use_rviz:=true
```

### 6.3 启动后验证

在另一个终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

ros2 topic hz /points_relay
ros2 topic hz /scan
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo lidar lidar_points
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /points_relay -v
```

预期：

- 点云和 `/scan` 约 10 Hz；
- `/map` 有数据；
- `lidar→lidar_points` yaw 约 +137°；
- `odom→base_link` 连续；
- `/points_relay` 只有一个发布者。

### 6.4 停止 mapping

优先在运行 launch 的终端按 `Ctrl+C`。

如果找不到原终端：

```bash
pgrep -af 'ros2 launch hypertron_ros2_bridge mapping.launch.py'
```

确认 PID 后，只向对应 launch 父进程发送：

```bash
kill -INT <mapping_launch_pid>
```

等待数秒并复核：

```bash
pgrep -af 'hypertron_driver_node|qos_relay.py|async_slam_toolbox_node|pointcloud_to_laserscan_node|rviz2'
```

不要在未确认 PID 和命令行的情况下使用宽泛的 `kill -9`。

## 7. RViz 点云观察方法

当前配置：

```bash
source /opt/ros/humble/setup.bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

ros2 run rviz2 rviz2 \
  -d /home/lee/hypertron-sw01-ros2-bridge/config/points_view.rviz
```

复核单个现实目标时建议：

```text
Fixed Frame = lidar
视角 = TopDownOrtho
显示 /points_relay
暂时关闭 Map 和 Grid
Point Size = 0.01～0.02
```

建图观察时：

```text
Fixed Frame = map 或 odom
显示 /points_relay、/scan、/map、TF
```

RViz 不是相机增强现实画面，无法靠肉眼直接证明点云覆盖现实世界。必须使用已知几何、卷尺、墙面、柱子或墙角进行数值复核。

## 8. 实时参照物检查工具

工具只订阅点云和 TF，不控制机器人：

```bash
source /opt/ros/humble/setup.bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

python3 /home/lee/hypertron-sw01-ros2-bridge/tools/live_cloud_reference_check.py \
  --target-frame lidar \
  --samples 5 \
  --angle-center-deg 0 \
  --angle-half-width-deg 5 \
  --range-min 0.7 \
  --range-max 1.3 \
  --z-min -0.25 \
  --z-max 0.50
```

主要输出：

```text
x、y、z：目标点簇在 target-frame 下的中位坐标
range：水平距离中位数
bearing_deg：atan2(y,x) 的方位角
range_q10_q50_q90：点簇距离分布
aggregate：多帧汇总结果
```

该工具只按 ROI 选择点。如果 ROI 中存在墙、柱子、家具等多个物体，结果会混合，必须先清场并合理缩小角度和距离窗口。

## 9. 明天优先执行：平墙距离和角度复核

### 9.1 为什么优先使用墙

柱子表面有曲率，柱面最近点、柱心和边缘距离不同；平整墙面在俯视点云中应形成直线，更适合验证比例、角度和地图几何。

选择：

- 平整、不透明、非玻璃、非镜面墙；
- 墙前清场；
- 机器人保持静止；
- 尽量让机器人 X 轴垂直墙面；
- 卷尺从 Odin1 雷达测量原点到墙面作垂直测量，不从脚尖、外壳或质心起算。

### 9.2 1 m 墙面

物理测量雷达到墙面垂直距离为 1.000 m，然后执行：

```bash
python3 /home/lee/hypertron-sw01-ros2-bridge/tools/live_cloud_reference_check.py \
  --target-frame lidar --samples 10 \
  --angle-center-deg 0 --angle-half-width-deg 10 \
  --range-min 0.7 --range-max 1.3 \
  --z-min -0.25 --z-max 0.50
```

对于正对平墙，中心区域应满足：

```text
x 中位数接近 1.000 m
y 分布左右展开
bearing 中心接近 0°
```

不要要求墙面所有点的径向 `range` 都等于 1 m；墙面边缘点的径向距离本来就比中心大，判断垂直墙距时主要看中心区域的 `x`。

### 9.3 2 m 墙面

```bash
python3 /home/lee/hypertron-sw01-ros2-bridge/tools/live_cloud_reference_check.py \
  --target-frame lidar --samples 10 \
  --angle-center-deg 0 --angle-half-width-deg 10 \
  --range-min 1.7 --range-max 2.3 \
  --z-min -0.25 --z-max 0.50
```

### 9.4 3 m 墙面

```bash
python3 /home/lee/hypertron-sw01-ros2-bridge/tools/live_cloud_reference_check.py \
  --target-frame lidar --samples 10 \
  --angle-center-deg 0 --angle-half-width-deg 10 \
  --range-min 2.7 --range-max 3.3 \
  --z-min -0.25 --z-max 0.50
```

记录：

| 物理墙距 | 点云 x 中位数 | 误差 | q10/q50/q90 | 结论 |
|---:|---:|---:|---:|---|
| 1.000 m | | | | |
| 2.000 m | | | | |
| 3.000 m | | | | |

拟合：

```text
measured = a × true_distance + b
```

- `a` 偏离 1：比例参数可能有误；
- `a≈1` 但 `b` 固定：检查卷尺起点、雷达原点和墙面基准；
- 误差不稳定：检查混入物体、反射、ROI 和丢包。

建议导航前目标：1～3 m 范围比例误差不超过约 2%，最大绝对误差控制在约 3～5 cm；最终阈值应结合厂商规格和目标场景确认。

## 10. 左右角度与 Y 轴复核

角度基准必须来自地面胶带、大号角尺或独立角度仪，不能只用待验证的 UDP 6101 里程计验证自己。

### 10.1 正前方

```bash
python3 tools/live_cloud_reference_check.py \
  --target-frame lidar --samples 10 \
  --angle-center-deg 0 --angle-half-width-deg 5 \
  --range-min 0.7 --range-max 1.3
```

预期 `bearing≈0°`、`y≈0`。

### 10.2 机器人向左转 10°，柱子保持不动

```bash
python3 tools/live_cloud_reference_check.py \
  --target-frame lidar --samples 10 \
  --angle-center-deg -10 --angle-half-width-deg 5 \
  --range-min 0.7 --range-max 1.3
```

预期柱子在机器人右侧：

```text
bearing≈-10°
y<0
```

### 10.3 机器人向右转 10°，柱子保持不动

```bash
python3 tools/live_cloud_reference_check.py \
  --target-frame lidar --samples 10 \
  --angle-center-deg 10 --angle-half-width-deg 5 \
  --range-min 0.7 --range-max 1.3
```

预期：

```text
bearing≈+10°
y>0
```

建议：正前方误差不超过 ±1°，左右 10° 测试误差各不超过约 ±2°，左右绝对值差不超过约 2°。

## 11. UDP 抓包和协议复核

驱动运行并已订阅雷达后：

```bash
sudo timeout 30 tcpdump -i eno1 -w /tmp/lidar6100.pcap udp port 6100
sudo timeout 30 tcpdump -i eno1 -w /tmp/odom6101.pcap udp port 6101
```

分析：

```bash
cd /home/lee/hypertron-sw01-ros2-bridge

python3 tools/calibration_analyze.py --cloud /tmp/lidar6100.pcap
python3 tools/calibration_analyze.py --odom /tmp/odom6101.pcap
```

运动标定必须在安全前提满足后分别抓取：

```bash
# 独立物理测量直线 1 m
sudo timeout 30 tcpdump -i eno1 -w /tmp/odom_forward_1m.pcap udp port 6101

# 独立物理测量原地旋转 90°
sudo timeout 30 tcpdump -i eno1 -w /tmp/odom_rotate_90deg.pcap udp port 6101
```

完成后分析并至少重复三次，不能仅用单次结果回填最终比例。

## 12. 正式建图准入与验收

必须先完成：

1. 墙面 1/2/3 m；
2. 左右角度；
3. 雷达离地高度和 Z 轴；
4. UDP 6101 多次直线和旋转；
5. 确认 `slam_toolbox` 不再持续丢弃新扫描，或证明丢弃仅为受控节流且地图持续更新；
6. 确认只有一个 TF 所有者发布 `map→odom`。

建图启动见第 6 节。使用遥控器低速覆盖环境：

- 先沿房间外圈；
- 避免高速转头和快速原地旋转；
- 重访起点触发回环；
- 观察墙面是否重影、弯曲、重复或跳变；
- 用卷尺已知墙距/走廊宽度对照地图。

保存 Nav2 二维地图：

```bash
mkdir -p /home/lee/maps
ros2 run nav2_map_server map_saver_cli -f /home/lee/maps/sw01_test
```

应生成：

```text
/home/lee/maps/sw01_test.yaml
/home/lee/maps/sw01_test.pgm
```

MindSLAM 的 `.bin` 地图与 Nav2 的 `.yaml + .pgm` 地图不是同一种格式，不能直接互换。

## 13. 保存地图后的定位和导航

### 13.1 AMCL 独立定位

```bash
source /opt/ros/humble/setup.bash
source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash

ros2 launch hypertron_ros2_bridge navigation.launch.py \
  map:=/home/lee/maps/sw01_test.yaml \
  use_amcl:=true
```

### 13.2 在线 SLAM 配合 Nav2

mapping 已运行时：

```bash
ros2 launch hypertron_ros2_bridge navigation.launch.py use_amcl:=false
```

### 13.3 自主导航前必须补齐

1. 机器人真实 footprint，不只使用占位 `robot_radius=0.35`；
2. `inflation_radius=0.55` 的实机通行安全性；
3. 点云/scan 的最小最大障碍高度；
4. 障碍探测和 raytrace 距离；
5. 最大速度、加速度、减速度；
6. 断点云、断里程计、TF 超时和控制断流的主动停车；
7. 遥控器抢权、软件急停、实体急停；
8. AMCL 初始位姿、重定位和绑架恢复；
9. 平地轮式模式下完成低速点到点、窄通道、动态障碍和恢复行为验收。

当前参数是导航骨架，不得因为 RViz 能发目标点就直接进行无人自主运行。

## 14. 如何获得机器人内部 Odin1 访问权限

### 14.1 先联系谁

第一联系人应是 **SW01 机器人厂家/集成商的技术支持或 FAE**，不是只处理购买、售后登记的普通客服。原因是：

- 他们决定 Odin1 如何连接机器人内部 RK3588；
- 他们拥有 RK3588 系统镜像、账号、服务配置和安全边界；
- ASTRALL UDP 6100/6101 是他们的转发接口；
- 只有他们能确认外部 Type-C、以太网、SSH、ADB 或串口是否开放。

SW01 手册标注的厂商是深圳市璇玑动力科技有限公司，手册中给出了官方客服电话 `400-099-2966`。联系时应明确要求转接“SW01 二次开发技术支持、软件 FAE 或嵌入式系统工程师”。

第二联系人是 **留形科技 Odin1 技术支持**。他们适合回答 Odin1 固件、官方驱动、MindSLAM 参数、USB 设备、地图格式和算法话题，但通常不能替机器人厂家提供 RK3588 登录账号。留形科技官方支持邮箱目前为 `support@manifoldtech.com`；发送前可在其官方网站再次确认。

### 14.2 不要只问“给我 3588 的 SSH 命令”

SSH 命令本身只是：

```bash
ssh -p <端口> <用户名>@<RK3588_IP>
```

真正需要厂家提供的是一套“RK3588 二次开发访问资料”，至少包括：

1. RK3588 的有线 IP 地址或发现方式；
2. SSH 是否启用、端口、用户名；
3. 密码、SSH 公钥开通方式或临时维护账号；
4. 是否允许 `sudo`，哪些操作被授权；
5. 外部 Type-C 是 ADB、USB Device、USB Host、视频输出还是调试复合接口；
6. 是否有串口控制台，以及串口电平、波特率、引脚；
7. RK3588 的 Ubuntu/系统版本、CPU 架构、ROS 版本；
8. Odin1 连接在哪个内部 USB 端口，是否能在 RK3588 上看到 `2207:0019`；
9. 当前占用 Odin1 的进程、Docker 容器或 systemd 服务名称；
10. 是否允许停止该服务并运行官方 `odin_ros_driver`；
11. 正确的停止、启动、重启和恢复步骤；
12. 系统镜像、恢复包和回滚方法；
13. 当前 Odin1 固件版本和配套驱动版本；
14. UDP 6100 对应 Odin 哪类点云；
15. UDP 6101 对应 Odin 哪个里程计输出、频率、坐标系、单位和时间戳；
16. 是否已有以太网 API 可切换 `custom_map_mode`、保存 `.bin` 地图、上传重定位地图；
17. 如果不开放 SSH，能否由厂家增加受控的 MindSLAM 模式/地图管理服务。

账号、密码和密钥应通过厂商认可的私密渠道获取，不要在群聊或公开工单中索取通用 root 密码。

### 14.3 给 SW01 厂家技术支持的可直接发送模板

```text
您好，我们正在对 Hypertron SW01（ASTRALL SDK 1.0.7）进行 ROS 2 Humble
二次开发。当前 PC 已通过 10.18.0.100 的有线接口正常获取 UDP 6100
AstrallPosCloudData 和 UDP 6101 AstrallOdometryData。

机器人内部空间模组为留形科技 Odin1。我们希望使用 Odin1 官方驱动验证
MindSLAM 的 Odometry / SLAM / Relocalization 模式，并保存、下载和加载 Odin
的 .bin 地图。目前外部以太网 SDK 文档只说明 6100/6101 数据转发，没有模式
切换和地图管理接口。

请协助转接 SW01 二次开发软件 FAE/嵌入式工程师，并提供或确认：
1. Odin1 与机器人内部 RK3588 的 USB 拓扑；
2. RK3588 的开发访问方式：IP、SSH 端口、用户名、公钥/授权流程，或者 ADB/
   串口/本地终端方式；
3. 外部 Type-C 接口是否可以进入 RK3588 或透传 Odin1 USB；
4. RK3588 系统版本、ROS 版本、Odin1 固件和驱动版本；
5. 当前占用 Odin1 的程序、容器或 systemd 服务名称，以及安全停止/恢复流程；
6. UDP 6100 对应 Odin 的 cloud_raw、cloud_slam 还是二次处理点云；
7. UDP 6101 对应普通/高频里程计中的哪一个，及其坐标系、单位、时间戳；
8. 是否已有以太网接口支持切换 MindSLAM 模式、保存地图、下载地图和加载
   重定位地图；若没有，是否可以提供受控接口；
9. 开发前的备份、恢复镜像和保修边界。

我们不会直接修改底层服务，希望先按贵司提供的安全流程做只读检查和备份。
谢谢。
```

### 14.4 给留形科技 Odin1 支持的可直接发送模板

```text
您好，我们在 Hypertron SW01 机器人中使用 Odin1 空间模组，正在进行 ROS 2
Humble 二次开发。机器人厂家通过 UDP 6100/6101 向外转发点云和里程计，但我们
尚未获得 Odin1 USB 的直接访问方式。

本地已有 odin_ros_driver v0.13.0。请协助确认：
1. Odin1 固件与 odin_ros_driver 的版本兼容矩阵；
2. USB 设备 2207:0019、USB 3.x、udev 和 ARM64/RK3588 的部署要求；
3. custom_map_mode=0/1/2 的正式使用流程；
4. save_map、.bin 地图下载、重定位地图上传和失败恢复流程；
5. /odin1/cloud_raw、/odin1/cloud_slam、普通/高频 odometry 的区别、坐标系、
   单位、频率和时间戳；
6. 一个 Odin1 是否允许厂家程序与官方驱动同时访问，还是必须独占 USB；
7. 在机器人内部 RK3588 上部署时推荐的系统、ROS 和资源配置。

如需设备序列号、固件日志或厂商集成信息，我们可以在机器人厂家授权后提供。
谢谢。
```

## 15. 获得 RK3588 权限后的只读检查顺序

厂商明确授权并提供登录方式后：

```bash
ssh -p <厂商提供端口> <厂商提供用户>@<RK3588_IP>
```

只读确认：

```bash
uname -a
cat /etc/os-release
uname -m
lsusb
lsusb -t
ps -ef | grep -Ei 'odin|mind|slam|lidar'
systemctl list-units --type=service | grep -Ei 'odin|mind|slam|lidar'
docker ps --format '{{.ID}} {{.Image}} {{.Names}}' 2>/dev/null
```

验收 Odin USB：

```text
2207:0019
```

此阶段不要停止任何服务、修改 udev、替换库或启动第二套 Odin 驱动。先把输出保存并与厂家确认独占访问和恢复方法。

## 16. 获得 Odin USB 独占访问后的 MindSLAM 流程

以下命令必须运行在能直接看到 Odin USB 的主机上，路径应按该主机实际工作空间调整。

### 16.1 里程计模式

`config/control_command.yaml`：

```yaml
custom_map_mode: 0
```

启动：

```bash
source /opt/ros/humble/setup.bash
source <odin_ws>/install/setup.bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 16.2 MindSLAM 建图

```yaml
custom_map_mode: 1
mapping_result_dest_dir: "/绝对路径/odin_maps"
mapping_result_file_name: "sw01_test.bin"
```

启动驱动并低速覆盖环境，完成后在驱动源码目录执行：

```bash
./set_param.sh save_map 1
```

不要连续快速触发保存；等待驱动确认地图生成和传输完成。

### 16.3 MindSLAM 重定位

```yaml
custom_map_mode: 2
relocalization_map_abs_path: "/绝对路径/odin_maps/sw01_test.bin"
```

然后启动同一驱动。初次测试尽量回到建图起点附近，并记录重定位成功时间、`map→odom`、轨迹连续性和失败恢复行为。

## 17. 端到端无地图导航的边界

留形科技的 SRU-Odin 是独立的学习型无地图导航方案，不等于当前 `slam_toolbox + Nav2`。接入 SW01 前至少需要：

1. ROS1/ROS2 兼容或完成 ROS2 移植；
2. 核对模型输入深度图、里程计、分辨率、频率和坐标系；
3. 核对输出速度对应的底盘模型；
4. 加入速度限幅、deadman、感知超时、控制权状态机和急停；
5. 先离线/仿真，再架空，再低速实机；
6. 不允许模型 `/cmd_vel` 直接绕过当前安全桥控制 SW01。

该路线不是明天点云标定和二维建图阶段的下一步，暂不启动。

## 18. 明天恢复工作的推荐顺序

1. 按第 5 节确认网络、单实例和安全状态；
2. 按第 6 节启动 mapping；
3. 确认点云、scan、TF 和地图数据；
4. 优先检查并解决 `slam_toolbox` queue full/TF 消息丢弃；
5. 实测 Odin 雷达原点到平墙的 1 m、2 m、3 m；
6. 使用第 8～9 节工具记录多帧结果；
7. 做左右 ±10° 角度和 Y 轴符号复核；
8. 实测雷达离地高度，调整点云高度裁剪；
9. 重复三次直线 1 m 和旋转 90° 的 UDP 6101 动态标定；
10. 只在结果稳定后回填参数并重新构建；
11. 建立一个已知尺寸的小区域地图，检查墙距、直角、闭环和重影；
12. 保存地图；
13. 完成 AMCL 重定位验收；
14. 实测机器人 footprint 和安全距离，最后才进入 Nav2 低速运动验收；
15. 并行联系 SW01 厂家 FAE 获取 RK3588/Odin1 的正式开发访问资料。

## 19. 记录表

| 日期 | 测试 | 物理真值 | ROS/点云结果 | 误差 | 参数变更 | 操作者 | 结论 |
|---|---|---:|---:|---:|---|---|---|
| 2026-08-18 | 正前方柱子重复性 | 实际距离未知 | 1.028 m，+0.17° | 距离不可判定 | yaw +137° 已在用 | | 方向初验通过 |
| | 平墙 1 m | | | | | | |
| | 平墙 2 m | | | | | | |
| | 平墙 3 m | | | | | | |
| | 左转 10° | | | | | | |
| | 右转 10° | | | | | | |
| | 雷达离地高度 | | | | | | |
| | odom 直线 1 m ×3 | | | | | | |
| | odom 旋转 90° ×3 | | | | | | |
| | 小区域地图尺寸 | | | | | | |
| | AMCL 重定位 | | | | | | |
