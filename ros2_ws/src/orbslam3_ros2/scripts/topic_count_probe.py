#!/usr/bin/env python3
# ================================================================
# topic_count_probe.py
# 설명: rosbag replay 중 좌·우 Image, IMU와 Clock 수신 수를 세어 atomic JSON으로 저장한다.
# 사용법:
#   ros2 run orbslam3_ros2 topic_count_probe.py --output /run/diagnostics/replay-counts.json
# ================================================================

import argparse
import json
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Imu


def _stamp_ns(stamp: Any) -> int:
    """ROS time을 정수 nanosecond로 변환한다."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class TopicCountProbe(Node):
    """replay sensor topic별 수와 timestamp 경계를 수집한다."""

    def __init__(self) -> None:
        super().__init__("orbslam3_replay_topic_count_probe")
        self.timestamps: dict[str, list[int]] = {
            "left": [],
            "right": [],
            "imu": [],
            "clock": [],
        }
        self.create_subscription(
            Image,
            "/rover/stereo/left/image_raw",
            lambda message: self.timestamps["left"].append(_stamp_ns(message.header.stamp)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/rover/stereo/right/image_raw",
            lambda message: self.timestamps["right"].append(_stamp_ns(message.header.stamp)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/rover/imu/data",
            lambda message: self.timestamps["imu"].append(_stamp_ns(message.header.stamp)),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Clock,
            "/clock",
            lambda message: self.timestamps["clock"].append(_stamp_ns(message.clock)),
            QoSProfile(depth=100),
        )

    def report(self) -> dict[str, object]:
        """topic별 count와 첫·마지막 timestamp를 반환한다."""
        return {
            "status": "complete",
            "counts": {name: len(values) for name, values in self.timestamps.items()},
            "timestamp_bounds_ns": {
                name: [values[0], values[-1]] if values else []
                for name, values in self.timestamps.items()
            },
        }


def _atomic_write_json(path: Path, data: dict[str, object]) -> None:
    """probe report를 임시 파일에서 원자적으로 공개한다."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> None:
    """SIGINT까지 replay topic을 수신하고 report를 기록한다."""
    parser = argparse.ArgumentParser(description="Count ORB-SLAM3 replay input topics")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = TopicCountProbe()
    try:
        rclpy.spin(node)
    finally:
        _atomic_write_json(args.output, node.report())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
