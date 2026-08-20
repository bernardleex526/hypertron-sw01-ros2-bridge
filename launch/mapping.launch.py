# ============================================================================
# 建图 launch 骨架：driver + pointcloud_to_laserscan + slam_toolbox（可选 rviz2）
#
# **安全红线**：`enable_tf_skeleton` 默认 false —— 不发布任何 TF。
# 当其置为 true 时，会发布三个静态 TF：
# - lidar→lidar_points：实机正前方 1 m 柱子标定的点云轴向修正；
# - lidar→base_link、base_link→imu_link：手册 3.5 节传感器外参。
# 手册平移值：
#   lidar: (0.3732, 0, 0.080)
#   imu:   (-0.00115, 0, 0.0595)
# 若实际安装与手册不一致，必须以实测为准修改。
#
# 说明：
# - pointcloud_to_laserscan 包在本机未安装时（需 sudo apt install
#   ros-humble-pointcloud-to-laserscan），ros2 launch 会直接报 executable
#   not found，属预期，本文件不做容错。
# - 驱动默认不发布 odom→base TF，slam 需要 odom→base_link（直连驱动
#   odom.publish_tf=false）。本骨架仅交付 TF 外参骨架；odom→base 定位链路
#   需在实机完成标定/里程计后补齐（见 REAL_MACHINE_CALIBRATION.md E 阶段）。
#
# 使用：ros2 launch hypertron_ros2_bridge mapping.launch.py [enable_tf_skeleton:=true]
# ============================================================================

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # 包 share 目录，用于定位 config 文件
    pkg = get_package_share_directory('hypertron_ros2_bridge')

    enable_tf_skeleton = LaunchConfiguration('enable_tf_skeleton')
    enable_odom_tf = LaunchConfiguration('enable_odom_tf')
    lidar_points_yaw = LaunchConfiguration('lidar_points_yaw')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_tf_skeleton',
            default_value='false',
            description=(
                '置 true 会发布 lidar→lidar_points 点云轴向修正，以及 '
                'lidar→base_link、base_link→imu_link 传感器外参。'
            ),
        ),
        DeclareLaunchArgument(
            'enable_odom_tf',
            default_value='false',
            description=(
                '是否让驱动把 /odom_lidar 发布为 odom→lidar TF。'
                '仅在 odometry.scale_verified=true 且外参标定后开启。'
            ),
        ),
        DeclareLaunchArgument(
            'lidar_points_yaw',
            default_value='2.391101075',
            description=(
                'lidar→lidar_points 的 yaw（弧度）。实机正前方柱子在 UDP 6100 '
                '原始坐标中的方位为 -137°，因此父帧到点云子帧为 +137°。'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start rviz2 for visualization.',
        ),

        # ------------------------------------------------------------------
        # QoS relay：把驱动 BEST_EFFORT 的 /points 转成 RELIABLE，供
        # pointcloud_to_laserscan 和 rviz2 使用（Humble rviz2 无法在 GUI 设置 QoS）。
        # ------------------------------------------------------------------
        ExecuteProcess(
            cmd=['python3', '/home/lee/hypertron-sw01-ros2-bridge/tools/qos_relay.py'],
            output='screen',
        ),

        # ------------------------------------------------------------------
        # 驱动节点（复用 driver.launch.py 的 Node 定义）
        # ------------------------------------------------------------------
        Node(
            package='hypertron_ros2_bridge',
            executable='hypertron_driver_node',
            name='hypertron_driver',
            output='screen',
            parameters=[
                pkg + '/config/driver_config.yaml',
                {'odom_lidar.publish_tf': ParameterValue(enable_odom_tf, value_type=bool)},
            ],
        ),

        # ------------------------------------------------------------------
        # pointcloud_to_laserscan：/points_relay → /scan
        # Humble 节点订阅 topic 名为 /cloud_in，发布 topic 名为 /scan；
        # 输入使用 QoS relay 转出的 /points_relay（RELIABLE），避免与驱动的
        # BEST_EFFORT /points 不兼容；输出无需 remap。
        # ------------------------------------------------------------------
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                pkg + '/config/laserscan_converter.yaml',
            ],
            remappings=[
                ('/cloud_in', '/points_relay'),
            ],
        ),

        # ------------------------------------------------------------------
        # slam_toolbox 在线异步建图（输出 /map + map→odom→base_link TF）
        # ------------------------------------------------------------------
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                pkg + '/config/slam_toolbox_online_async.yaml',
            ],
        ),

        # ------------------------------------------------------------------
        # TF 骨架（默认关闭）：
        #   odom→lidar：驱动根据 UDP 6101 发布，原点在雷达、轴向与机体一致；
        #   lidar→lidar_points：UDP 6100 原始点云轴向修正；
        #   lidar→base_link、base_link→imu_link：传感器与质心外参。
        #
        # 正前方约 1 m 柱子的原始点云主方向实测为 -137°。将原始点坐标旋转
        # +137° 后，柱子中心为 x≈1.022 m、y≈0.006 m，因此这里发布父帧
        # lidar 到子帧 lidar_points 的 +137° yaw。同一雷达原点，平移为 0。
        #
        # 外参来自《Hypertron-SW01 软件开发手册-中文版》3.5 节：
        # 雷达相对机器人质心坐标 = (373.2, 0, 80) mm。
        # 驱动已发布 odom -> lidar，因此这里必须发布 lidar -> base_link，
        # 组成 odom -> lidar -> base_link 链。
        # lidar -> base_link 的平移是手册值的相反数（(-0.3732, 0, -0.080)）。
        # 姿态先按与机体同向处理。
        # 语法来自 tf2_ros static_transform_publisher 可执行程序的新式参数。
        # ------------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar_to_points',
            output='screen',
            condition=IfCondition(enable_tf_skeleton),
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--yaw', lidar_points_yaw,
                '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'lidar', '--child-frame-id', 'lidar_points',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_lidar_to_base',
            output='screen',
            condition=IfCondition(enable_tf_skeleton),
            arguments=[
                '--x', '-0.3732', '--y', '0.0', '--z', '-0.08',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'lidar', '--child-frame-id', 'base_link',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            output='screen',
            condition=IfCondition(enable_tf_skeleton),
            arguments=[
                '--x', '-0.00115', '--y', '0.0', '--z', '0.0595',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
        ),

        # ------------------------------------------------------------------
        # 可选 rviz2（use_rviz:=true）
        # ------------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', pkg + '/config/points_view.rviz'],
        ),
    ])
