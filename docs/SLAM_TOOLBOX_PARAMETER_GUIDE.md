# SW01 / Odin1 slam_toolbox 参数说明

适用链路：UDP 6100 点云 → `pointcloud_to_laserscan` → `/scan` →
`slam_toolbox`，以及 UDP 6101 → `odom→lidar→base_link` TF。

当前主配置：`config/slam_toolbox_online_async.yaml`。角度参数的单位均为弧度。
以下“增大/减小”的效果必须通过实机地图重影、回环跳变、CPU占用和丢帧日志
共同判断，不能只看 RViz 中是否有地图。

## 本次已调整

| 参数 | 当前值 | 意义 |
|---|---:|---|
| `minimum_time_interval` | 0.1 s | 两次被处理扫描的最小时间间隔 |
| `minimum_travel_distance` | 0.10 m | 平移达到该距离后允许创建新位姿图节点 |
| `minimum_travel_heading` | 0.10 rad（约5.7°） | 旋转达到该角度后允许创建新节点 |
| `scan_buffer_size` | 20 | 保存用于近邻匹配/回环搜索的历史扫描数量 |
| `scan_queue_size` | 50 | 等待TF的LaserScan消息队列，只用于吸收短时抖动 |
| `transform_timeout` | 0.5 s | 等待扫描时刻TF的最长时间 |
| `tf_buffer_duration` | 60 s | slam_toolbox内部TF缓存长度 |
| 转换器 `queue_size` | 50 | 点云等待目标帧TF的队列长度 |
| 转换器 `transform_tolerance` | 0.3 s | 点云转LaserScan时的TF等待裕量 |

## 输入、帧和输出节奏

| 参数 | 当前值 | 意义与调整影响 |
|---|---:|---|
| `odom_frame` | `odom` | 局部连续里程计坐标系；只能有一条有效TF链通向 `base_link` |
| `map_frame` | `map` | 全局地图坐标系；建图时由slam_toolbox发布 `map→odom` |
| `base_frame` | `base_link` | 扫描匹配使用的机器人基准帧 |
| `scan_topic` | `/scan` | 输入二维LaserScan话题 |
| `throttle_scans` | 1 | 每N帧处理一帧；增大可降CPU，但快速运动时更易丢匹配 |
| `transform_publish_period` | 0.02 s | `map→odom`发布周期；0表示不发布 |
| `map_update_interval` | 5.0 s | OccupancyGrid更新时间；减小只让RViz更及时，不会提高SLAM精度 |
| `resolution` | 0.05 m | 地图栅格分辨率；减小更细但耗内存/CPU，且不能补偿雷达噪声 |
| `min_laser_range` | 0.11 m | 略高于 `/scan.range_min=0.1`，避免浮点边界警告；近场少用1 cm对建图无实质影响 |
| `max_laser_range` | 12.0 m | 用于建图的最大距离；过大时远距离噪声和稀疏回波会降低匹配稳定性 |
| `use_map_saver` | true | 启用地图保存相关能力 |
| `enable_interactive_mode` | true | 允许RViz交互式图编辑/位姿操作；正式无人运行可关闭 |
| `debug_logging` | false | 打开详细日志；只建议短时诊断使用 |

## 连续扫描匹配和近邻约束

| 参数 | 当前值 | 意义与调整影响 |
|---|---:|---|
| `use_scan_matching` | true | 使用激光扫描匹配；关闭后主要依赖里程计，不适合当前建图 |
| `use_scan_barycenter` | true | 使用扫描点重心辅助匹配 |
| `link_scan_maximum_distance` | 1.5 m | 与附近历史扫描建立约束的最大距离；过小易断链，过大增加错误匹配和计算量 |
| `link_match_minimum_response_fine` | 0.1 | 相邻扫描精匹配最低响应；提高更保守，降低会接受更弱匹配 |
| `scan_buffer_maximum_scan_distance` | 10.0 m | 扫描缓冲中参与匹配的最大空间距离 |
| `correlation_search_space_dimension` | 0.5 m | 普通扫描匹配平移搜索窗口；增大能容忍更差里程计，但更慢、更易误配 |
| `correlation_search_space_resolution` | 0.01 m | 普通匹配搜索网格分辨率；减小更细但更耗CPU |
| `correlation_search_space_smear_deviation` | 0.1 m | 相关栅格的模糊尺度；不是角度参数 |

## 回环检测和回环约束

| 参数 | 当前值 | 意义与调整影响 |
|---|---:|---|
| `do_loop_closing` | true | 是否自动检测并加入回环约束 |
| `loop_search_maximum_distance` | 3.0 m | 在多大距离内寻找历史回环候选；增大覆盖更广但计算量和误回环风险上升 |
| `loop_match_minimum_chain_size` | 10 | 候选区域至少需要多少连续历史扫描；降低更容易闭环，也更容易误闭环 |
| `loop_match_maximum_variance_coarse` | 3.0 | 粗匹配允许的最大方差；减小会更严格 |
| `loop_match_minimum_response_coarse` | 0.35 | 粗回环最低响应；提高可减少错误候选 |
| `loop_match_minimum_response_fine` | 0.45 | 精回环最低响应；提高更保守，降低可能出现地图突然跳变 |
| `loop_search_space_dimension` | 8.0 m | 回环相关搜索窗口大小；增大会显著增加计算量 |
| `loop_search_space_resolution` | 0.05 m | 回环搜索网格分辨率 |
| `loop_search_space_smear_deviation` | 0.03 m | 回环相关栅格模糊尺度 |

空旷、对称、长直走廊环境不要先降低回环响应阈值。回环只能在重访历史区域后
修正累计误差，不能解决机器人当前处于无特征区域时的不可观测问题。

## 扫描匹配代价与角度搜索

| 参数 | 当前值 | 意义与调整影响 |
|---|---:|---|
| `distance_variance_penalty` | 0.5 | 对偏离里程计平移预测的惩罚 |
| `angle_variance_penalty` | 1.0 | 对偏离里程计角度预测的惩罚 |
| `fine_search_angle_offset` | 0.00349 rad | 精匹配角度步进/偏移量，约0.2° |
| `coarse_search_angle_offset` | 0.349 rad | 粗匹配角度搜索范围，约20° |
| `coarse_angle_resolution` | 0.0349 rad | 粗匹配角分辨率，约2° |
| `minimum_angle_penalty` | 0.9 | 角度惩罚下限 |
| `minimum_distance_penalty` | 0.5 | 距离惩罚下限 |
| `use_response_expansion` | true | 匹配响应不足时扩展搜索 |
| `min_pass_through` | 2 | 栅格被观测多少次后才更可信；增大可抑制瞬时噪声但建图变慢 |
| `occupancy_threshold` | 0.1 | 判定栅格占用的阈值 |

`distance_variance_penalty`、`angle_variance_penalty`与各角度搜索参数属于高级参数。
只有在TF不丢帧、距离/角度标定和 `/scan` 有效波束比例已通过后再调整。

## Ceres位姿图优化

| 参数 | 当前值 | 意义 |
|---|---|---|
| `solver_plugin` | `solver_plugins::CeresSolver` | 位姿图非线性优化后端 |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | 稀疏线性求解器 |
| `ceres_preconditioner` | `SCHUR_JACOBI` | 预条件器 |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | 信赖域优化策略 |
| `ceres_loss_function` | `None` | 鲁棒损失函数；改变前必须验证异常约束处理效果 |

这些求解器参数通常不是空旷场地丢定位的首要调节项。

## PointCloud转LaserScan几何参数

配置文件：`config/laserscan_converter.yaml`。

| 参数 | 当前值 | 意义与调整影响 |
|---|---:|---|
| `target_frame` | `base_link` | 投影前把点云转换到该帧 |
| `min_height` / `max_height` | -0.5 / 0.5 m | 参与二维投影的高度带；过窄会丢墙/柱，过宽会引入地面、桌面和腿部 |
| `angle_min` / `angle_max` | -π / π | LaserScan角度范围；应与Odin1实际视场一致 |
| `angle_increment` | 0.0058 rad | LaserScan角分辨率；过细会产生大量 `inf`，不会凭空增加真实分辨率 |
| `range_min` / `range_max` | 0.1 / 12.0 m | 接受的距离范围；应与slam_toolbox的范围参数协调 |
| `scan_time` | 0.1 s | 一帧扫描周期；应接近实测 `/scan` 频率 |
| `use_inf` | true | 无回波波束用 `inf` 表达 |
| `inf_epsilon` | 1.0 m | `inf`替代值相关裕量 |

## 推荐调参顺序

1. 先确认 `/scan`、`odom→base_link` TF时间连续，日志不再持续出现
   `queue is full`、`earlier than all data in transform cache`。
2. 统计 `/scan` 有效波束比例，并标定高度带、实际视场和有效距离。
3. 调节点密度：`minimum_time_interval`、`minimum_travel_distance`、
   `minimum_travel_heading`。
4. 再调普通扫描匹配窗口和约束阈值。
5. 最后用明确的闭环路线调整回环参数；每次只改一组并保存对比地图。
