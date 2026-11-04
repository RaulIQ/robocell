#!/usr/bin/env python3
"""Exercise the installed adapter with controlled scans and delayed real TF."""

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Quaternion, Transform, TransformStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path)
parser.add_argument("--domain", type=int, default=86)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
rclpy.init(domain_id=args.domain)
node = rclpy.create_node("isolated_scan_verification")
clock = node.create_publisher(Clock, "/clock", 10)
source = node.create_publisher(LaserScan, "/scan", 10)
transforms = TransformBroadcaster(node)
messages = []
node.create_subscription(
    LaserScan, "/scan/slam", messages.append, qos_profile_sensor_data
)
checks = []


def pump(duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        clock.publish(Clock(clock=Time(seconds=5).to_msg()))
        rclpy.spin_once(node, timeout_sec=0.01)


def until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        pump(0.01)
    if not predicate():
        raise TimeoutError(
            "Isolated adapter verification exceeded its bounded wait"
        )


def scan(second, count, offset):
    return LaserScan(
        header=Header(
            stamp=Time(seconds=second).to_msg(), frame_id="base_scan"
        ),
        angle_min=offset,
        angle_max=offset - (count - 1) * math.tau / count,
        angle_increment=-math.tau / count,
        time_increment=0.000476,
        scan_time=count * 0.000476,
        range_min=0.15,
        range_max=12.0,
        ranges=[1.73] * count,
    )


def history(second):
    transforms.sendTransform(
        [
            TransformStamped(
                header=Header(
                    stamp=Time(seconds=moment).to_msg(), frame_id="odom"
                ),
                child_frame_id="base_scan",
                transform=Transform(rotation=Quaternion(w=1.0)),
            )
            for moment in (second, second + 0.5)
        ]
    )


def check(name, condition):
    checks.append({"name": name, "passed": bool(condition)})
    print(json.dumps(checks[-1]), flush=True)
    if not condition:
        raise AssertionError(f"Isolated adapter check failed: {name}")


with (args.output / "adapter.log").open("w") as stream:
    process = subprocess.Popen(
        [
            "ros2",
            "run",
            "robocell_sim",
            "scan.py",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "transform_timeout:=0.5",
            "--log-level",
            "fatal",
        ],
        env={**os.environ, "ROS_DOMAIN_ID": str(args.domain)},
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        until(
            lambda: (
                source.get_subscription_count() == 1
                and node.count_publishers("/scan/slam") == 1
            ),
            8,
        )
        pump(0.3)
        source.publish(scan(1, 381, 0.013))
        pump(0.15)
        check("waits_for_delayed_tf", len(messages) == 0)
        history(1)
        until(lambda: len(messages) == 1, 2)
        check("recovers_delayed_tf_with_frozen_clock", len(messages) == 1)
        source.publish(scan(2, 382, -0.017))
        pump(0.8)
        history(2)
        pump(0.2)
        check("expired_scan_cannot_reappear_after_late_tf", len(messages) == 1)
        source.publish(scan(3, 382, 0.027))
        history(3)
        until(lambda: len(messages) == 2, 2)
        check("fresh_scan_recovers_after_expiry", len(messages) == 2)
        source.publish(scan(4, 381, -0.031))
        history(4)
        until(lambda: len(messages) == 3, 2)
        check(
            "variable_turns_keep_one_grid",
            len(
                {
                    (
                        len(message.ranges),
                        message.angle_min,
                        message.angle_max,
                        message.angle_increment,
                        message.time_increment,
                    )
                    for message in messages
                }
            )
            == 1,
        )
        check("no_raw_republication", node.count_publishers("/scan") == 1)
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        node.context.try_shutdown()
        (args.output / "checks.json").write_text(
            json.dumps(
                {
                    "checks": checks,
                    "output_stamps": [
                        Time.from_msg(message.header.stamp).nanoseconds
                        for message in messages
                    ],
                },
                indent=2,
            )
        )
