from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='hypertron_ros2_bridge',
            executable='hypertron_driver_node',
            name='hypertron_driver',
            output='screen',
            parameters=[
                get_package_share_directory('hypertron_ros2_bridge')
                + '/config/driver_config.yaml'
            ],
        ),
    ])
