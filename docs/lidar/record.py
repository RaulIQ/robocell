#!/usr/bin/env python3
"""Record A1 acquisition timing while pausing the workshop and UART reader."""

import argparse
import contextlib
import hashlib
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path

import can
import rclpy
import rosbag2_py
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path)
parser.add_argument("--domain", type=int, default=84)
parser.add_argument("--channel", default="vcan0")
args = parser.parse_args()
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=False)
with can.Bus(interface="socketcan", channel=args.channel) as bus:
    if bus.recv(0.3) is not None:
        raise RuntimeError("The selected CAN channel already carries traffic")
rclpy.init(domain_id=args.domain)
node = rclpy.create_node("lidar_timing_verification")
writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(
        uri=str(args.output / "bag"), storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions("", ""),
)
types = {
    "/scan": ("sensor_msgs/msg/LaserScan", LaserScan),
    "/clock": ("rosgraph_msgs/msg/Clock", Clock),
    "/motor_driver/state": ("std_msgs/msg/String", String),
    "/gyro/state": ("std_msgs/msg/String", String),
}
latest = {}
scans = []
checks = []
process = None
reader = -1
started = time.monotonic()
for index, (topic, (typename, _)) in enumerate(types.items()):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=index, name=topic, type=typename, serialization_format="cdr"
        )
    )


def receive(topic, message):
    latest[topic] = message
    writer.write(topic, serialize_message(message), time.time_ns())
    if topic == "/scan":
        stamp = (
            message.header.stamp.sec * 1_000_000_000
            + message.header.stamp.nanosec
        )
        period = round(message.time_increment * 1_000_000_000)
        scans.append(
            {
                "wall": time.monotonic() - started,
                "stamp": stamp,
                "finish": stamp + len(message.ranges) * period,
                "points": len(message.ranges),
                "period": period,
                "angle": message.angle_increment,
                "frame": message.header.frame_id,
            }
        )


for topic, (_, cls) in types.items():
    node.create_subscription(
        cls,
        topic,
        lambda message, name=topic: receive(name, message),
        qos_profile_action_status_default
        if topic in {"/motor_driver/state", "/gyro/state"}
        else qos_profile_sensor_data,
    )
client = node.create_client(SetBool, "/sim/pause")


def pump():
    if process.poll() is not None:
        raise RuntimeError(
            "Workshop exited before timing verification finished"
        )
    rclpy.spin_once(node, timeout_sec=0.005)


def until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        pump()
    if not predicate():
        raise TimeoutError("Lidar verification exceeded its bounded wait")


def advance(duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        pump()


def pause(value):
    if not client.wait_for_service(timeout_sec=3):
        raise TimeoutError("Workshop pause service did not appear")
    future = client.call_async(SetBool.Request(data=value))
    until(future.done, 5)
    if not future.result().success:
        raise RuntimeError("Workshop rejected the pause request")


def clock():
    return (
        latest["/clock"].clock.sec * 1_000_000_000
        + latest["/clock"].clock.nanosec
    )


def check(name, passed, **evidence):
    result = {"name": name, "passed": bool(passed), **evidence}
    checks.append(result)
    print(json.dumps(result), flush=True)
    if not passed:
        raise AssertionError(f"Lidar timing check failed: {name}")


with (args.output / "launch.log").open("w") as stream:
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "robocell_sim",
            "workshop.launch.xml",
            "headless:=true",
            "teleop:=false",
            "rviz:=false",
            "seed:=422",
            f"can_channel:={args.channel}",
            "left_node:=17",
            "right_node:=29",
            f"lidar_port:={args.output / 'lidar'}",
        ],
        env={
            **os.environ,
            "ROS_DOMAIN_ID": str(args.domain),
            "MUJOCO_GL": "egl",
        },
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        until(
            lambda: (
                len(scans) >= 10
                and "/gyro/state" in latest
                and latest["/gyro/state"].data == "ready"
                and "/motor_driver/state" in latest
                and latest["/motor_driver/state"].data == "enabled"
            ),
            30,
        )
        reader = int(
            re.search(
                r"\[lidar.py-\d+\]: process started with pid \[(\d+)\]",
                (args.output / "launch.log").read_text(),
            ).group(1)
        )
        for cycle in range(3):
            if cycle == 1:
                os.kill(reader, signal.SIGSTOP)
                advance(1.1)
            pause(True)
            advance(0.3)
            frozen = clock()
            if cycle == 1:
                os.kill(reader, signal.SIGCONT)
            beginning = len(scans)
            advance(0.8)
            check(
                f"pause_{cycle}",
                clock() == frozen
                and all(scan["finish"] <= frozen for scan in scans[beginning:]),
                clock=frozen,
                drained=len(scans) - beginning,
                reader_suspended=cycle == 1,
            )
            pause(False)
            resumed = time.monotonic()
            beginning = len(scans)
            until(
                lambda: (
                    len(scans) >= beginning + 6 and scans[-1]["stamp"] > frozen
                ),
                10,
            )
            check(
                f"resume_{cycle}",
                scans[-1]["stamp"] > frozen,
                latency=time.monotonic() - resumed,
                scans=len(scans) - beginning,
            )
        check(
            "acquisition_order",
            all(
                after["stamp"] >= before["finish"]
                for before, after in zip(scans, scans[1:])
            ),
            scans=len(scans),
        )
        check(
            "wire_geometry_and_period",
            all(
                scan["period"] == 476_000
                and scan["angle"] < 0
                and scan["frame"] == "base_scan"
                for scan in scans
            ),
            points=sorted({scan["points"] for scan in scans}),
        )
        check(
            "one_scan_publisher",
            [
                endpoint.node_name
                for endpoint in node.get_publishers_info_by_topic("/scan")
            ]
            == ["lidar"],
        )
    finally:
        with contextlib.suppress(ProcessLookupError):
            if reader > 0:
                os.kill(reader, signal.SIGCONT)
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        writer.close()
        node.context.try_shutdown()
        (args.output / "scans.json").write_text(json.dumps(scans, indent=2))
        (args.output / "checks.json").write_text(
            json.dumps(
                {
                    "checks": checks,
                    "scans": len(scans),
                    "duration": time.monotonic() - started,
                    "launch_returncode": process.returncode,
                    "bag_sha256": hashlib.sha256(
                        (args.output / "bag/bag_0.db3").read_bytes()
                    ).hexdigest(),
                },
                indent=2,
            )
        )
