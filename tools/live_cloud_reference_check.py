#!/usr/bin/env python3
"""Measure a known reference object from the live SW01 point cloud.

The tool transforms ``/points_relay`` into a requested frame, selects a
range/angle/height region of interest, and reports robust coordinate and range
statistics.  It is intentionally read-only: it does not request control
authority or publish robot commands.
"""

from __future__ import annotations

import argparse
import math
import statistics
import time
from typing import Iterable, Sequence, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


Point = Tuple[float, float, float, float, float]


def percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))
    return ordered[index]


def rotate_vector(x: float, y: float, z: float, q) -> Tuple[float, float, float]:
    """Rotate a vector by a geometry_msgs quaternion (xyzw)."""
    ux, uy, uz, scalar = q.x, q.y, q.z, q.w
    dot_uv = ux * x + uy * y + uz * z
    dot_uu = ux * ux + uy * uy + uz * uz
    cross_x = uy * z - uz * y
    cross_y = uz * x - ux * z
    cross_z = ux * y - uy * x
    scale = scalar * scalar - dot_uu
    return (
        2.0 * dot_uv * ux + scale * x + 2.0 * scalar * cross_x,
        2.0 * dot_uv * uy + scale * y + 2.0 * scalar * cross_y,
        2.0 * dot_uv * uz + scale * z + 2.0 * scalar * cross_z,
    )


class ReferenceChecker(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("live_cloud_reference_check")
        self.args = args
        self.clouds: list[PointCloud2] = []
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            PointCloud2,
            args.topic,
            self._on_cloud,
            qos_profile_sensor_data,
        )

    def _on_cloud(self, message: PointCloud2) -> None:
        if len(self.clouds) < self.args.samples:
            self.clouds.append(message)


def select_points(
    cloud: PointCloud2,
    transform,
    args: argparse.Namespace,
) -> list[Point]:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    selected: list[Point] = []

    for raw in point_cloud2.read_points(
        cloud, field_names=("x", "y", "z"), skip_nans=True
    ):
        x, y, z = map(float, raw)
        x, y, z = rotate_vector(x, y, z, rotation)
        x += translation.x
        y += translation.y
        z += translation.z
        horizontal_range = math.hypot(x, y)
        bearing = math.degrees(math.atan2(y, x))
        angle_error = (bearing - args.angle_center_deg + 180.0) % 360.0 - 180.0
        if not (args.range_min <= horizontal_range <= args.range_max):
            continue
        if abs(angle_error) > args.angle_half_width_deg:
            continue
        if not (args.z_min <= z <= args.z_max):
            continue
        selected.append((x, y, z, horizontal_range, bearing))

    return selected


def medians(points: Iterable[Point]) -> Point:
    values = list(points)
    return tuple(statistics.median(point[index] for point in values) for index in range(5))  # type: ignore[return-value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/points_relay")
    parser.add_argument("--target-frame", default="lidar")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--angle-center-deg", type=float, default=0.0)
    parser.add_argument("--angle-half-width-deg", type=float, default=5.0)
    parser.add_argument("--range-min", type=float, default=0.5)
    parser.add_argument("--range-max", type=float, default=1.5)
    parser.add_argument("--z-min", type=float, default=-0.25)
    parser.add_argument("--z-max", type=float, default=0.50)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples < 1:
        raise SystemExit("--samples must be at least 1")
    if args.range_min >= args.range_max:
        raise SystemExit("--range-min must be less than --range-max")

    rclpy.init()
    node = ReferenceChecker(args)
    deadline = time.monotonic() + args.timeout
    while len(node.clouds) < args.samples and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.clouds:
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"no messages received from {args.topic}")

    sample_results: list[Point] = []
    total_selected = 0
    for index, cloud in enumerate(node.clouds, start=1):
        try:
            transform = node.buffer.lookup_transform(
                args.target_frame,
                cloud.header.frame_id,
                Time(),
                timeout=Duration(seconds=2.0),
            )
        except Exception as error:
            print(f"sample={index} transform_error={error}")
            continue
        selected = select_points(cloud, transform, args)
        if not selected:
            print(f"sample={index} selected_points=0")
            continue
        result = medians(selected)
        ranges = [point[3] for point in selected]
        total_selected += len(selected)
        sample_results.append(result)
        print(
            f"sample={index} selected_points={len(selected)} "
            f"x={result[0]:.3f} y={result[1]:.3f} z={result[2]:.3f} "
            f"range={result[3]:.3f} bearing_deg={result[4]:.2f} "
            f"range_q10_q50_q90=({percentile(ranges, 0.10):.3f},"
            f"{statistics.median(ranges):.3f},{percentile(ranges, 0.90):.3f})"
        )

    node.destroy_node()
    rclpy.shutdown()
    if not sample_results:
        raise SystemExit("no points matched the configured reference-object ROI")

    aggregate = medians(sample_results)
    print(
        f"aggregate samples={len(sample_results)} selected_points={total_selected} "
        f"target_frame={args.target_frame} x={aggregate[0]:.3f} "
        f"y={aggregate[1]:.3f} z={aggregate[2]:.3f} "
        f"range={aggregate[3]:.3f} bearing_deg={aggregate[4]:.2f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
