# ============================================================================
# 建图 launch 骨架：driver + pointcloud_to_laserscan + slam_toolbox（可选 rviz2）
#
# **安全红线**：`enable_tf_skeleton` 默认 false —— 不发布任何 TF。
# 当其置为 true 时，会以**占位外参（0,0,0,0,0,0）**发布 base_link→lidar 与
# base_link→imu_link 两个静态 TF。这些是占位值，未实机标定（见
# docs/REAL_MACHINE_CALIBRATION.md B8/B10），**禁止在真实环境开启**。
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 包 share 目录，用于定位 config 文件
    pkg = get_package_share_directory('hypertron_ros2_bridge')

    enable_tf_skeleton = LaunchConfiguration('enable_tf_skeleton')
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_tf_skeleton',
            default_value='false',
            description=(
                '危险：置 true 会以占位外参(0,0,0,0,0,0)发布 base_link→lidar 与 '
                'base_link→imu_link 静态 TF。未标定禁止开启（见 '
                'docs/REAL_MACHINE_CALIBRATION.md B8/B10）。'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start rviz2 for visualization.',
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
            ],
        ),

        # ------------------------------------------------------------------
        # pointcloud_to_laserscan：/points → /scan
        # Humble 节点订阅 topic 名为 /cloud_in，发布 topic 名为 /scan；
        # 因此只需要把输入 /cloud_in 重映射到本驱动的 /points，输出无需
        # remap（不存在 /scan_out 这个 topic）。
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
                ('/cloud_in', '/points'),
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
        # TF 骨架（默认关闭）：base_link→lidar 与 base_link→imu_link
        # 占位平移/姿态全部为 0（0,0,0,0,0,0）—— 未标定禁止开启。
        # 语法来自 tf2_ros static_transform_publisher 可执行程序的新式参数。
        # ------------------------------------------------------------------
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_lidar',
            output='screen',
            condition=IfCondition(enable_tf_skeleton),
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'lidar',
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            output='screen',
            condition=IfCondition(enable_tf_skeleton),
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
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
        ),
    ])
