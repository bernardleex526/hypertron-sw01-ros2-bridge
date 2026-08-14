# ============================================================================
# 导航 launch 骨架：封装 nav2_bringup 的 navigation_launch.py
#
# 前置：
#   需同时运行 mapping.launch（slam_toolbox 提供 map→odom 定位 + /map 话题），
#   或后续自行接入 map_server + AMCL 提供定位。导航本身不负责定位。
#
# 安全：导航速度已在 config/nav2_params.yaml 中保守限制（max_vel_x 0.3、
# max_vel_theta 0.5、acc 0.5/1.0）；所有几何参数为占位值，未标定禁止实机使用。
#
# 用法：ros2 launch hypertron_ros2_bridge navigation.launch.py map:=<绝对路径>/map.yaml
# ============================================================================

from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg = get_package_share_directory('hypertron_ros2_bridge')

    map_arg = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            # 我们自己的 params_file 骨架
            'params_file': pkg + '/config/nav2_params.yaml',
            # nav2_bringup navigation_launch.py 本身不消费 map 参数；
            # 这里声明并传入 map 仅作为导航骨架应关联地图的显式记录。
            # 实际 /map 话题由同时运行的 mapping.launch（slam）或后续
            # map_server/AMCL 提供。
            'map': map_arg,
            'use_sim_time': use_sim_time,
            'use_composition': 'False',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            # 必填：用户提供已保存的 map.yaml 绝对路径（骨架阶段仅作记录）。
            # 示例: map:=/home/lee/maps/sw01_map.yaml
            description='Absolute path to the saved map yaml (served by slam or map_server).',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true.',
        ),
        navigation_launch,
    ])
