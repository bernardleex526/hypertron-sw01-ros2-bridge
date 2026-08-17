# ============================================================================
# 导航 launch：封装 nav2_bringup 的 navigation_launch.py，并可选启动
# map_server + AMCL（localization_launch.py）用于独立导航。
#
# 前置：
# - 若 use_amcl:=true：需提供 map:=<绝对路径>/map.yaml，并已有标定后的
#   odom→base_link TF（例如驱动 odom_lidar.publish_tf + lidar→base_link）。
# - 若 use_amcl:=false：需同时运行 mapping.launch（slam_toolbox 提供
#   map→odom 定位 + /map 话题）。
#
# 安全：导航速度已在 config/nav2_params.yaml 中保守限制（max_vel_x 0.3、
# max_vel_theta 0.5、acc 0.5/1.0）；所有几何参数为占位值，未标定禁止实机使用。
#
# 用法：
#   # 独立导航（map_server + AMCL）
#   ros2 launch hypertron_ros2_bridge navigation.launch.py \
#     map:=<绝对路径>/map.yaml use_amcl:=true
#   # 配合 SLAM 建图节点在线导航
#   ros2 launch hypertron_ros2_bridge navigation.launch.py use_amcl:=false
# ============================================================================

from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg = get_package_share_directory('hypertron_ros2_bridge')

    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_amcl = LaunchConfiguration('use_amcl')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            # 我们自己的 params_file 骨架
            'params_file': pkg + '/config/nav2_params.yaml',
            'use_sim_time': use_sim_time,
            'use_composition': 'False',
            'autostart': 'true',
        }.items(),
    )

    # 可选独立定位：map_server + AMCL。使用 use_amcl:=true 时启动，
    # 否则由外部 mapping.launch（SLAM）提供定位。
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_arg,
            'params_file': pkg + '/config/nav2_params.yaml',
            'use_sim_time': use_sim_time,
            'use_composition': 'False',
            'autostart': 'true',
        }.items(),
        condition=IfCondition(use_amcl),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='',
            description=(
                'Absolute path to a saved map.yaml. Required when use_amcl:=true; '
                'ignored when use_amcl:=false.'
            ),
        ),
        DeclareLaunchArgument(
            'use_amcl',
            default_value='false',
            description=(
                'Start map_server + AMCL for standalone localization. '
                'Set false when using mapping.launch (SLAM) for localization.'
            ),
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true.',
        ),
        localization_launch,
        navigation_launch,
    ])
