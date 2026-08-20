# 2026-08-20 更新说明：Odin1 点云标定、时间同步与 slam_toolbox 稳定性

## 更新目标

本次更新将 SW01 转发的 Odin1 UDP 6100 点云和 UDP 6101 里程计接入可持续
运行的二维建图链路，修复 `slam_toolbox` 因时间戳/TF不同步持续出现的：

```text
Message Filter dropping message ... queue is full
the timestamp on the message is earlier than all the data in the transform cache
```

同时根据正前方约 1 m 柱子的实机数据修正点云轴向和比例，并降低
`slam_toolbox` 位姿图节点采样间隔，使配置更适合低速室内轮式模式。

## 主要变化

### 1. UDP 6100 点云组帧语义

- 实机抓包确认 `total` 是一帧总点数。
- `index` 是本包在帧内的起始点偏移（`0, 50, 100, ...`），不是包序号。
- 组帧器按点区间精确覆盖 `[0, total)` 判断完整帧。
- 拒绝越界、重叠、冲突和超过容量上限的数据包。

### 2. 点云坐标和TF链

- UDP 6100 点云 frame 改为 `lidar_points`。
- UDP 6101 里程计 child frame 保持 `lidar`。
- 新增 `lidar→lidar_points` 轴向修正，默认 yaw：

```text
2.391101075 rad = +137°
```

- 实机正前方约 1 m 柱子修正后约为 `x=1.025 m, y=0.005 m`。
- 使用手册外参发布 `lidar→base_link=(-0.3732, 0, -0.080)`。
- 当前有效TF链：

```text
map → odom → lidar → base_link
                 └→ lidar_points
```

正式导航前仍需完成侧向目标、2/3 m距离、雷达高度和Z轴复核。

### 3. 设备时间同步

UDP 6100/6101中的时间字段是同一设备单调纳秒时钟，不是Unix时间。驱动现在：

1. 使用UDP 6101接收时刻估计设备时钟到ROS时钟的偏移；
2. 使用平滑和跳变检测抑制网络抖动，并处理设备重启；
3. UDP 6100点云复用同一时钟映射；
4. `/odom_lidar` 与 `odom→lidar` TF复用完全相同的时间戳；
5. 防止映射结果落到ROS接收时刻之后。

这解决了分别对点云、里程计和TF调用 `now()` 导致的扫描时刻无法查询对应TF。

### 4. QoS和RViz

- 新增 `tools/qos_relay.py`，把传感器 BEST_EFFORT 话题转为RELIABLE：
  - `/points` → `/points_relay`
  - `/odom_lidar` → `/odom_lidar_relay`
  - `/imu/data` → `/imu_relay`
- `pointcloud_to_laserscan` 改为订阅 `/points_relay`。
- 新增 `config/points_view.rviz`，预置点云、LaserScan、Map和TF显示。

### 5. 标定和诊断工具

- `tools/calibration_analyze.py`：分析UDP 6100/6101 pcap结构、尺度、范围和覆盖。
- `tools/live_cloud_reference_check.py`：实时用柱子/墙面检查距离、方向和高度。
- 新增标定交接、完整SOP和参数说明文档。

## 当前参数配置

### 驱动：`config/driver_config.yaml`

| 参数 | 当前值 | 意义 |
|---|---:|---|
| `lidar.point_cloud_port` | 6100 | Odin1点云UDP端口 |
| `lidar.odometry_port` | 6101 | Odin1里程计UDP端口 |
| `lidar.source_ip` | `10.18.0.100` | 机器人服务端地址 |
| `lidar.point_position_scale` | 0.0000191 | 点坐标原始整数到米的比例 |
| `lidar.odom_position_scale` | 0.0000009474 | 里程计位置原始整数到米的比例 |
| `lidar.odom_quaternion_order` | `xyzw` | 6101四元数分量顺序 |
| `lidar.frame_id` | `lidar_points` | UDP 6100点云坐标系 |
| `lidar.frame_timeout_ms` | 300 | 不完整点云帧超时 |
| `lidar.max_parallel_frames` | 4 | 最大并行组帧数量 |
| `lidar.max_points_per_frame` | 2000000 | 单帧点数上限 |
| `lidar.max_packets_per_frame` | 40000 | 单帧数据包数量上限 |
| `odom_lidar.parent_frame` | `odom` | 里程计父坐标系 |
| `odom_lidar.child_frame` | `lidar` | 里程计子坐标系 |
| `odom_lidar.publish_tf` | false | 默认不发布TF，由launch显式开启 |
| `odometry.scale_verified` | true | 当前标定比例允许显式发布6101 TF |

### 点云转LaserScan：`config/laserscan_converter.yaml`

| 参数 | 当前值 | 意义 |
|---|---:|---|
| `target_frame` | `base_link` | 投影前目标坐标系 |
| `min_height` / `max_height` | -0.5 / 0.5 m | 参与二维投影的高度范围，仍需实测收紧 |
| `angle_min` / `angle_max` | -π / π | 输出LaserScan角度范围 |
| `angle_increment` | 0.0058 rad | 角度步进 |
| `range_min` / `range_max` | 0.1 / 12.0 m | 有效测距范围 |
| `scan_time` | 0.1 s | 标称扫描周期 |
| `transform_tolerance` | 0.3 s | TF等待裕量 |
| `queue_size` | 50 | 点云等待TF的消息队列 |
| `use_inf` | true | 无回波波束使用无穷大 |

### slam_toolbox：`config/slam_toolbox_online_async.yaml`

| 参数 | 当前值 | 意义 |
|---|---:|---|
| `mode` | `mapping` | 在线二维建图模式 |
| `resolution` | 0.05 m | 地图栅格分辨率 |
| `map_update_interval` | 5.0 s | OccupancyGrid更新周期 |
| `min_laser_range` | 0.11 m | 避开0.1 m浮点边界警告 |
| `max_laser_range` | 12.0 m | 最大建图距离 |
| `minimum_time_interval` | 0.1 s | 两次处理扫描的最小时间间隔 |
| `minimum_travel_distance` | 0.10 m | 创建节点的最小平移 |
| `minimum_travel_heading` | 0.10 rad | 创建节点的最小旋转（约5.7°） |
| `transform_timeout` | 0.5 s | 等待扫描时刻TF的超时 |
| `tf_buffer_duration` | 60.0 s | TF缓存长度 |
| `scan_queue_size` | 50 | slam_toolbox等待TF的扫描队列 |
| `scan_buffer_size` | 20 | 用于约束搜索的历史扫描缓存 |
| `do_loop_closing` | true | 启用回环检测 |
| `loop_search_maximum_distance` | 3.0 m | 回环候选搜索距离 |
| `loop_match_minimum_chain_size` | 10 | 回环候选最小连续扫描数量 |
| `loop_match_minimum_response_coarse` | 0.35 | 粗回环最低响应 |
| `loop_match_minimum_response_fine` | 0.45 | 精回环最低响应 |
| `correlation_search_space_dimension` | 0.5 m | 普通扫描匹配平移搜索窗口 |
| `loop_search_space_dimension` | 8.0 m | 回环相关搜索窗口 |

其余参数及增大/减小影响见 `docs/SLAM_TOOLBOX_PARAMETER_GUIDE.md`。

## 启动命令

```bash
cd /home/lee/hypertron-sw01-ros2-bridge
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch hypertron_ros2_bridge mapping.launch.py \
  enable_tf_skeleton:=true \
  enable_odom_tf:=true \
  use_rviz:=true
```

`enable_tf_skeleton` 和 `enable_odom_tf` 默认均为 `false`，防止未标定环境误用。

## 验证结果

2026-08-19在实机上连续订阅20秒：

```text
/points:          10.275 Hz
/scan:            10.269 Hz
/odom_lidar:      10.259 Hz
odom→lidar TF:    10.262 Hz
/map:              0.200 Hz
点云-最近里程计时间差: 0.000000 s
TF-里程计消息时间差:   0.000000000 s
```

隔离ROS域测试：

```text
lidar_stream      Passed
package_contract  Passed
driver_node       Passed
100% tests passed
```

新日志未再出现 `queue is full`、TF缓存过旧或LaserScan最小距离警告。

## 尚未完成的验收

- 侧向目标和Y轴正负方向；
- 1/2/3 m多距离线性度；
- 雷达光学中心离地高度、Z轴和地面回波；
- UDP 6101重复直线和90°旋转误差；
- 空旷场地、长走廊和完整闭环路线的地图误差；
- AMCL全局重定位；
- Nav2避障、失联停车和实机自主导航。

本次验证证明数据链、时间同步、TF查询和二维地图持续更新可运行，不等同于完整
定位精度或自主导航安全验收。
