#!/usr/bin/env python3
"""Relay driver sensor topics with BEST_EFFORT QoS to RELIABLE topics for rviz2.

ROS 2 Humble's rviz2 does not expose per-display QoS settings in the GUI, so
it cannot subscribe to the driver's BEST_EFFORT sensor topics. This node
republishes the same messages on RELIABLE topics that rviz2 can display.

Run:
    source /opt/ros/humble/setup.bash
    source /home/lee/hypertron-sw01-ros2-bridge/install/setup.bash
    python3 /home/lee/hypertron-sw01-ros2-bridge/tools/qos_relay.py

Then in rviz2 add:
    /points_relay    (PointCloud2)
    /odom_lidar_relay (Odometry)
    /imu_relay       (IMU)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry


def reliable_volatile_qos() -> QoSProfile:
    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class QosRelay(Node):
    def __init__(self) -> None:
        super().__init__("qos_relay")
        sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = reliable_volatile_qos()

        self.pub_points = self.create_publisher(PointCloud2, "/points_relay", pub_qos)
        self.sub_points = self.create_subscription(
            PointCloud2, "/points", self._on_points, sub_qos
        )

        self.pub_odom = self.create_publisher(Odometry, "/odom_lidar_relay", pub_qos)
        self.sub_odom = self.create_subscription(
            Odometry, "/odom_lidar", self._on_odom, sub_qos
        )

        self.pub_imu = self.create_publisher(Imu, "/imu_relay", pub_qos)
        self.sub_imu = self.create_subscription(
            Imu, "/imu/data", self._on_imu, sub_qos
        )

        self.get_logger().info(
            "QoS relay started: "
            "/points -> /points_relay, "
            "/odom_lidar -> /odom_lidar_relay, "
            "/imu/data -> /imu_relay"
        )

    def _on_points(self, msg: PointCloud2) -> None:
        self.pub_points.publish(msg)

    def _on_odom(self, msg: Odometry) -> None:
        self.pub_odom.publish(msg)

    def _on_imu(self, msg: Imu) -> None:
        self.pub_imu.publish(msg)


def main() -> None:
    rclpy.init()
    node = QosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()