#!/usr/bin/env python3
"""Compare raw and compensated gyro fusion on the same workshop sensors."""

import argparse
import json
import math
import os
import signal
import sqlite3
import subprocess
import time
from pathlib import Path

import can
import numpy as np
import rclpy
import rosbag2_py
import yaml
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path)
parser.add_argument("--seed", type=int, default=422)
parser.add_argument("--domain", type=int, default=96)
parser.add_argument("--channel", default="vcan0")
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
root = Path(__file__).resolve().parents[2]
with can.Bus(interface="socketcan", channel=args.channel) as bus:
    if bus.recv(0.5) is not None:
        raise RuntimeError("Selected CAN channel already carries traffic")
parameters = yaml.safe_load(
    (root / "src/robocell_sim/config/ekf.yaml").read_text()
)["ekf_filter_node"]["ros__parameters"]
parameters.update(imu0="/imu", publish_tf=False)
(args.output / "baseline.yaml").write_text(
    yaml.safe_dump({"gyro_baseline": {"ros__parameters": parameters}})
)
writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(
        uri=str(args.output / "bag"), storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions("", ""),
)
types = {
    "/imu": ("sensor_msgs/msg/Imu", Imu),
    "/imu/calibrated": ("sensor_msgs/msg/Imu", Imu),
    "/joint_states": ("sensor_msgs/msg/JointState", JointState),
    "/ground_truth/odom": ("nav_msgs/msg/Odometry", Odometry),
    "/wheel/odometry": ("nav_msgs/msg/Odometry", Odometry),
    "/odometry/filtered": ("nav_msgs/msg/Odometry", Odometry),
    "/odometry/uncompensated": ("nav_msgs/msg/Odometry", Odometry),
    "/gyro/state": ("std_msgs/msg/String", String),
    "/tf_static": ("tf2_msgs/msg/TFMessage", TFMessage),
    "/clock": ("rosgraph_msgs/msg/Clock", Clock),
    "/cmd_vel": ("geometry_msgs/msg/Twist", Twist),
}
for index, (name, (typename, _)) in enumerate(types.items()):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=index, name=name, type=typename, serialization_format="cdr"
        )
    )
rclpy.init(domain_id=args.domain)
node = rclpy.create_node("gyro_experiment")
latest = {}


def receive(name, message):
    latest[name] = message
    writer.write(name, serialize_message(message), time.time_ns())


for name, (_, cls) in types.items():
    qos = (
        qos_profile_action_status_default
        if name in {"/gyro/state", "/tf_static"}
        else qos_profile_sensor_data
        if name in {"/imu", "/imu/calibrated"}
        else 100
    )
    node.create_subscription(
        cls, name, lambda message, topic=name: receive(topic, message), qos
    )
publisher = node.create_publisher(Twist, "/cmd_vel", 1)
processes = []
logs = []
environment = {
    **os.environ,
    "ROS_DOMAIN_ID": str(args.domain),
    "MUJOCO_GL": "egl",
}
commands = [
    [
        "ros2",
        "launch",
        "robocell_sim",
        "workshop.launch.xml",
        "headless:=true",
        "teleop:=false",
        "rviz:=false",
        f"seed:={args.seed}",
        f"can_channel:={args.channel}",
        "left_node:=17",
        "right_node:=29",
        f"lidar_port:={args.output / 'lidar'}",
    ],
    [
        "/opt/ros/jazzy/lib/robot_localization/ekf_node",
        "--ros-args",
        "-r",
        "__node:=gyro_baseline",
        "--params-file",
        str(args.output / "baseline.yaml"),
        "-r",
        "odometry/filtered:=/odometry/uncompensated",
    ],
]


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def pump():
    rclpy.spin_once(node, timeout_sec=0.002)
    if any(process.poll() is not None for process in processes):
        raise RuntimeError("A gyro experiment process exited before completion")


def until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        pump()
    if not predicate():
        raise TimeoutError(
            "Gyro experiment exceeded its startup or motion deadline"
        )


try:
    for index, command in enumerate(commands):
        stream = (args.output / f"process-{index}.log").open("w")
        logs.append(stream)
        processes.append(
            subprocess.Popen(
                command,
                cwd=root,
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        )
    until(
        lambda: (
            all(
                name in latest
                for name in (
                    "/gyro/state",
                    "/imu/calibrated",
                    "/odometry/filtered",
                    "/odometry/uncompensated",
                    "/ground_truth/odom",
                )
            )
            and latest["/gyro/state"].data == "ready"
        ),
        20,
    )
    until(lambda: stamp(latest["/ground_truth/odom"]) >= 10, 30)
    for duration, angular in [(4, -0.4), (4, 0.0), (4, 0.4), (4, 0.0)]:
        target = stamp(latest["/ground_truth/odom"]) + duration
        sent = -1.0
        deadline = time.monotonic() + 20
        while stamp(latest["/ground_truth/odom"]) < target:
            if time.monotonic() > deadline:
                raise TimeoutError("Gyro turn exceeded its wall-time deadline")
            if time.monotonic() - sent >= 0.05:
                publisher.publish(Twist(angular=Vector3(z=angular)))
                sent = time.monotonic()
            pump()
    publisher.publish(Twist())
finally:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
    for stream in logs:
        stream.close()
    writer.close()
    node.context.try_shutdown()
streams = {
    name: []
    for name in (
        "/ground_truth/odom",
        "/odometry/filtered",
        "/odometry/uncompensated",
    )
}
imu = {"/imu": {}, "/imu/calibrated": {}}
with sqlite3.connect(args.output / "bag/bag_0.db3") as connection:
    for name, payload in connection.execute(
        "SELECT topics.name, messages.data FROM messages JOIN topics ON topics.id=messages.topic_id ORDER BY messages.timestamp, messages.id"
    ):
        if name in streams:
            message = deserialize_message(payload, Odometry)
            q = message.pose.pose.orientation
            streams[name].append(
                (
                    stamp(message),
                    math.atan2(
                        2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z),
                    ),
                )
            )
        if name in imu:
            message = deserialize_message(payload, Imu)
            imu[name].setdefault(stamp(message), message.angular_velocity.z)
truth = np.array(streams["/ground_truth/odom"])
truth[:, 1] = np.unwrap(truth[:, 1])
metrics = {
    "seed": args.seed,
    "stationary_duration_s": 10,
    "route": [[4, -0.4], [4, 0.0], [4, 0.4], [4, 0.0]],
}
for name in ("/odometry/filtered", "/odometry/uncompensated"):
    values = np.array(streams[name])
    values = values[
        (values[:, 0] >= max(3.0, truth[0, 0])) & (values[:, 0] <= truth[-1, 0])
    ]
    error = np.angle(
        np.exp(
            1j
            * (
                values[:, 1]
                + math.pi / 2
                - np.interp(values[:, 0], truth[:, 0], truth[:, 1])
            )
        )
    )
    np.savetxt(
        args.output / (name.rsplit("/", 1)[1] + ".csv"),
        np.column_stack((values[:, 0], np.degrees(error))),
        delimiter=",",
        header="simulation_time,heading_error_deg",
        comments="",
    )
    stationary = error[values[:, 0] <= 10]
    metrics[name] = {
        "heading_rms_deg": float(np.degrees(np.sqrt(np.mean(error**2)))),
        "heading_final_deg": float(np.degrees(error[-1])),
        "stationary_final_deg": float(np.degrees(stationary[-1])),
    }
first = min(imu["/imu/calibrated"])
metrics["first_corrected_stamp"] = first
metrics["estimated_z_bias"] = float(
    np.mean(
        [
            imu["/imu"][instant] - value
            for instant, value in imu["/imu/calibrated"].items()
            if instant in imu["/imu"]
        ]
    )
)
metrics["truth_heading_range_deg"] = float(np.degrees(np.ptp(truth[:, 1])))
metrics["passed"] = bool(
    first >= 2
    and metrics["truth_heading_range_deg"] > 60
    and abs(metrics["/odometry/filtered"]["stationary_final_deg"]) < 0.2
    and metrics["/odometry/filtered"]["heading_rms_deg"]
    < metrics["/odometry/uncompensated"]["heading_rms_deg"] / 5
)
(args.output / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
print(json.dumps(metrics, indent=2))
if not metrics["passed"]:
    raise SystemExit(
        "Gyro compensation did not pass the stationary and turning checks"
    )
