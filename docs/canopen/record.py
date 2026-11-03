#!/usr/bin/env python3
"""Record the actual workshop launch, its CAN traffic and recovery checks."""

import argparse
import contextlib
import csv
import gzip
import json
import math
import os
import re
import secrets
import signal
import struct
import subprocess
import time
from pathlib import Path

import can
import rclpy
import rosbag2_py
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage

arguments = argparse.ArgumentParser(description=__doc__)
arguments.add_argument("output", type=Path)
arguments.add_argument("--channel", default="vcan0")
arguments.add_argument("--domain", type=int, default=secrets.randbelow(70) + 20)
arguments.add_argument("--seed", type=int, default=121)
options = arguments.parse_args()
options.output.mkdir(parents=True, exist_ok=False)
root = Path(__file__).resolve().parents[2]
numbers = (17, 29)
latest = {}
counts = {}
states = []
frames = []
emergencies = []
results = []
positions = []
source = None
sent = -1.0
start = time.monotonic()
process = None
driver = -1
launchlog = options.output / "launch.log"
traffic = gzip.open(options.output / "can.log.gz", "wt")
writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(
        uri=str(options.output / "bag"), storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions("", ""),
)
types = {
    "/cmd_vel": ("geometry_msgs/msg/Twist", Twist),
    "/keyboard/cmd_vel": ("geometry_msgs/msg/TwistStamped", TwistStamped),
    "/clock": ("rosgraph_msgs/msg/Clock", Clock),
    "/joint_states": ("sensor_msgs/msg/JointState", JointState),
    "/scan": ("sensor_msgs/msg/LaserScan", LaserScan),
    "/imu": ("sensor_msgs/msg/Imu", Imu),
    "/wheel/odometry": ("nav_msgs/msg/Odometry", Odometry),
    "/odometry/filtered": ("nav_msgs/msg/Odometry", Odometry),
    "/ground_truth/odom": ("nav_msgs/msg/Odometry", Odometry),
    "/motor_driver/state": ("std_msgs/msg/String", String),
    "/tf": ("tf2_msgs/msg/TFMessage", TFMessage),
    "/tf_static": ("tf2_msgs/msg/TFMessage", TFMessage),
}
for index, (topic, (name, _)) in enumerate(types.items()):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=index, name=topic, type=name, serialization_format="cdr"
        )
    )
rclpy.init(domain_id=options.domain)
node = rclpy.create_node("canopen_acceptance")
publishers = {
    "ros": node.create_publisher(Twist, "/cmd_vel", 1),
    "keyboard": node.create_publisher(TwistStamped, "/keyboard/cmd_vel", 1),
}
clients = {
    "pause": node.create_client(SetBool, "/sim/pause"),
    "retry": node.create_client(Trigger, "/motor_driver/retry"),
    "recover": node.create_client(Trigger, "/motor_driver/recover"),
}


def receive(topic, message):
    latest[topic] = message
    counts[topic] = counts.get(topic, 0) + 1
    writer.write(topic, serialize_message(message), time.time_ns())
    if topic == "/motor_driver/state":
        states.append((time.monotonic() - start, message.data))
    if topic in {"/ground_truth/odom", "/wheel/odometry", "/odometry/filtered"}:
        positions.append(
            (
                topic,
                message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.twist.twist.linear.x,
                message.twist.twist.angular.z,
            )
        )


for topic, (_, cls) in types.items():
    qos = (
        qos_profile_action_status_default
        if topic in {"/motor_driver/state", "/tf_static"}
        else qos_profile_sensor_data
        if topic in {"/scan", "/imu"}
        else 100
    )
    node.create_subscription(
        cls, topic, lambda message, name=topic: receive(name, message), qos
    )


def publish():
    global sent
    if source is not None and time.monotonic() - sent >= 0.05:
        kind, linear, angular = source
        message = Twist(linear=Vector3(x=linear), angular=Vector3(z=angular))
        publishers[kind].publish(
            TwistStamped(
                header=Header(
                    stamp=Time(nanoseconds=time.time_ns()).to_msg(),
                    frame_id="base_link",
                ),
                twist=message,
            )
            if kind == "keyboard"
            else message
        )
        sent = time.monotonic()


def capture():
    for _ in range(256):
        message = bus.recv(0)
        if message is None:
            break
        frame = (
            time.monotonic() - start,
            message.arbitration_id,
            bytes(message.data),
        )
        frames.append(frame)
        traffic.write(
            f"({time.time():.6f}) {options.channel} {message.arbitration_id:03X}#{bytes(message.data).hex().upper()}\n"
        )
        if message.arbitration_id in {0x80 + number for number in numbers}:
            emergencies.append(
                (
                    frame[0],
                    message.arbitration_id - 0x80,
                    int.from_bytes(message.data[:2], "little"),
                )
            )


def pump():
    publish()
    rclpy.spin_once(node, timeout_sec=0.002)
    capture()
    if process is not None and process.poll() is not None:
        raise RuntimeError("Workshop launch exited during the recording")


def wait(predicate, timeout, context):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        pump()
    if not predicate():
        raise TimeoutError(context)


def advance(duration, command):
    global source
    source = command
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        pump()


def check(name, success, **evidence):
    result = {
        "name": name,
        "passed": bool(success),
        "wall": time.monotonic() - start,
        **evidence,
    }
    results.append(result)
    print(json.dumps(result), flush=True)
    if not success:
        raise AssertionError(f"CANopen check failed: {name}")


def call(name, request):
    client = clients[name]
    if not client.wait_for_service(timeout_sec=3.0):
        raise TimeoutError(f"Service {name} did not appear")
    future = client.call_async(request)
    wait(future.done, 5.0, f"Service {name} did not answer")
    if not future.result().success:
        raise RuntimeError(f"Service {name} rejected its request")


def state():
    return (
        latest["/motor_driver/state"].data
        if "/motor_driver/state" in latest
        else "missing"
    )


def value(number, index):
    beginning = time.monotonic() - start
    bus.send(
        can.Message(
            arbitration_id=0x600 + number,
            data=struct.pack("<BHB4x", 0x40, index, 0),
            is_extended_id=False,
        ),
        timeout=0.01,
    )
    matches = lambda: [
        frame
        for frame in frames
        if frame[0] >= beginning
        and frame[1] == 0x580 + number
        and len(frame[2]) == 8
        and int.from_bytes(frame[2][1:3], "little") == index
        and frame[2][3] == 0
    ]
    wait(
        lambda: bool(matches()),
        0.5,
        f"Drive {number} did not answer diagnostic SDO {index:04X}",
    )
    payload = matches()[-1][2]
    if payload[0] == 0x80:
        raise RuntimeError(f"Drive {number} aborted diagnostic SDO {index:04X}")
    return int.from_bytes(
        payload[4 : 4 + {0x4F: 1, 0x4B: 2, 0x43: 4}[payload[0]]],
        "little",
        signed=index == 0x60FF,
    )


def targets():
    return [value(number, 0x60FF) for number in numbers]


with (
    can.Bus(interface="socketcan", channel=options.channel) as bus,
    launchlog.open("w") as log,
):
    if bus.recv(0.5) is not None:
        raise RuntimeError("The selected CAN channel already carries traffic")
    environment = {**os.environ, "ROS_DOMAIN_ID": str(options.domain)}
    command = [
        "ros2",
        "launch",
        "robocell_sim",
        "workshop.launch.xml",
        "headless:=true",
        "teleop:=false",
        "rviz:=false",
        f"can_channel:={options.channel}",
        "left_node:=17",
        "right_node:=29",
        f"seed:={options.seed}",
        f"lidar_port:={options.output / 'lidar'}",
    ]
    process = subprocess.Popen(
        command,
        cwd=root,
        env=environment,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        wait(
            lambda: (
                state() == "enabled"
                and all(
                    topic in latest
                    for topic in (
                        "/joint_states",
                        "/scan",
                        "/imu",
                        "/wheel/odometry",
                        "/odometry/filtered",
                        "/ground_truth/odom",
                        "/clock",
                    )
                )
            ),
            10.0,
            "The full ROS chain did not become ready",
        )
        driver = int(
            re.search(
                r"\[motor_driver.py-\d+\]: process started with pid \[(\d+)\]",
                launchlog.read_text(),
            ).group(1)
        )
        feedback = [
            endpoint.node_name
            for endpoint in node.get_publishers_info_by_topic("/joint_states")
        ]
        check(
            "one_joint_publisher", feedback == ["motor_driver"], nodes=feedback
        )
        advance(0.2, ("ros", 0.0, 0.0))
        advance(4.0, ("keyboard", -0.08, 0.0))
        rear = targets()
        check(
            "keyboard_collision_guard",
            rear == [0, 0]
            and max(abs(value) for value in latest["/joint_states"].velocity)
            < 0.02,
            drive_targets=rear,
            measured_wheel_velocities=list(latest["/joint_states"].velocity),
            requested=-0.08,
        )
        for duration, linear, angular in [
            (3.0, 0.08, 0.0),
            (3.0, 0.0, -0.4),
            (4.0, 0.08, 0.0),
            (3.0, 0.0, 0.4),
            (0.5, 0.08, 0.0),
            (1.0, -0.06, 0.0),
        ]:
            advance(duration, ("ros", linear, angular))
        check(
            "moving_route",
            any(
                abs(row[4]) > 0.03
                for row in positions
                if row[0] == "/ground_truth/odom"
            ),
            samples=len(positions),
        )
        advance(0.4, None)
        wait(
            lambda: state() == "enabled" and targets() == [0, 0],
            1.0,
            "Stale body commands did not zero the enabled drives",
        )
        stopped = targets()
        check(
            "stale_command_stop",
            stopped == [0, 0]
            and abs(latest["/ground_truth/odom"].twist.twist.linear.x) < 0.01,
            drive_targets=stopped,
            measured_body_velocity=latest[
                "/ground_truth/odom"
            ].twist.twist.linear.x,
            state=state(),
        )
        advance(1.1, ("ros", 0.05, 0.0))
        check(
            "fresh_input_resumes",
            state() == "enabled"
            and any(targets())
            and latest["/ground_truth/odom"].twist.twist.linear.x > 0.02,
            state=state(),
            measured_body_velocity=latest[
                "/ground_truth/odom"
            ].twist.twist.linear.x,
        )
        call("pause", SetBool.Request(data=True))
        advance(0.15, ("ros", 0.05, 0.0))
        stamp = (
            latest["/clock"].clock.sec * 1000000000
            + latest["/clock"].clock.nanosec
        )
        advance(0.35, ("ros", 0.05, 0.0))
        paused = targets()
        frozen = (
            latest["/clock"].clock.sec * 1000000000
            + latest["/clock"].clock.nanosec
        )
        check(
            "pause_services_can_without_motion",
            stamp == frozen and paused == [0, 0] and state() == "enabled",
            clock_before=stamp,
            clock_after=frozen,
            drive_targets=paused,
            state=state(),
        )
        resumed = time.monotonic() - start
        call("pause", SetBool.Request(data=False))
        wait(
            lambda: any(
                instant > resumed and condition == "enabled"
                for instant, condition in states
            ),
            3.0,
            "Resume did not repeat verified enable",
        )
        advance(0.3, ("ros", 0.05, 0.0))
        check("resume_reenables", state() == "enabled", state=state())
        suspended = time.monotonic() - start
        os.kill(driver, signal.SIGSTOP)
        advance(0.24, ("ros", 0.05, 0.0))
        os.kill(driver, signal.SIGCONT)
        source = None
        wait(
            lambda: state().startswith("stopped:"),
            2.0,
            "Watchdog fault did not stop the host",
        )
        advance(0.08, None)
        faults = [value(number, 0x603F) for number in numbers]
        stopped = targets()
        latency = {
            number: 1000
            * (
                min(
                    instant
                    for instant, node, code in emergencies
                    if node == number
                    and code == 0xFF01
                    and instant >= suspended
                )
                - max(
                    instant
                    for instant, identifier, payload in frames
                    if identifier == 0x200 + number
                    and instant
                    < min(
                        stamp
                        for stamp, node, code in emergencies
                        if node == number
                        and code == 0xFF01
                        and stamp >= suspended
                    )
                )
            )
            for number in numbers
        }
        check(
            "command_watchdog_fault",
            faults == [0xFF01, 0xFF01]
            and stopped == [0, 0]
            and max(abs(value) for value in latest["/joint_states"].velocity)
            < 0.02,
            faults=faults,
            drive_targets=stopped,
            watchdog_delay_ms=latency,
            measured_wheel_velocities=list(latest["/joint_states"].velocity),
        )
        retry = time.monotonic() - start
        call("retry", Trigger.Request())
        wait(
            lambda: any(
                instant > retry and condition.startswith("stopped:")
                for instant, condition in states
            ),
            3.0,
            "Normal retry did not preserve the fault",
        )
        faults = [value(number, 0x603F) for number in numbers]
        check(
            "retry_does_not_clear_fault",
            faults == [0xFF01, 0xFF01],
            faults=faults,
        )
        recovered = time.monotonic() - start
        call("recover", Trigger.Request())
        wait(
            lambda: any(
                instant > recovered and condition == "enabled"
                for instant, condition in states
            ),
            5.0,
            "Explicit recovery did not enable the drives",
        )
        advance(0.06, None)
        cleared = [value(number, 0x603F) for number in numbers]
        stopped = targets()
        check(
            "recovery_discards_cached_motion",
            cleared == [0, 0] and stopped == [0, 0],
            faults=cleared,
            drive_targets=stopped,
        )
        advance(0.7, ("ros", 0.05, 0.0))
        check(
            "recovery_accepts_fresh_motion",
            any(targets())
            and latest["/ground_truth/odom"].twist.twist.linear.x > 0.02,
            state=state(),
            measured_body_velocity=latest[
                "/ground_truth/odom"
            ].twist.twist.linear.x,
        )
        restarted = time.monotonic() - start
        bus.send(
            can.Message(
                arbitration_id=0, data=b"\x81\x11", is_extended_id=False
            ),
            timeout=0.01,
        )
        wait(
            lambda: any(
                instant > restarted and condition.startswith("stopped:")
                for instant, condition in states
            ),
            2.0,
            "Drive restart did not stop the host",
        )
        source = None
        advance(0.05, None)
        stopped = targets()
        check(
            "drive_restart_stops_both",
            stopped == [0, 0],
            drive_targets=stopped,
            state=state(),
        )
        retried = time.monotonic() - start
        call("retry", Trigger.Request())
        wait(
            lambda: any(
                instant > retried and condition == "enabled"
                for instant, condition in states
            ),
            5.0,
            "Drive restart did not permit verified retry",
        )
        advance(0.06, None)
        check(
            "restart_discards_cached_motion", targets() == [0, 0], state=state()
        )
        advance(0.4, ("ros", 0.0, 0.0))
        check(
            "all_required_topics_recorded",
            all(counts.get(topic, 0) for topic in types),
            counts=counts,
        )
    finally:
        shutdown = time.monotonic() - start
        if driver > 0:
            with contextlib.suppress(ProcessLookupError):
                os.kill(driver, signal.SIGCONT)
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        for _ in range(8):
            capture()
        words = {
            number: [
                struct.unpack("<Hi", payload)[0]
                for instant, identifier, payload in frames
                if instant >= shutdown
                and identifier == 0x200 + number
                and len(payload) == 6
                and struct.unpack("<Hi", payload)[1] == 0
            ]
            for number in numbers
        }
        results.append(
            {
                "name": "clean_shutdown_zero_disable",
                "passed": all(
                    2 in sequence and 0 in sequence
                    for sequence in words.values()
                ),
                "wall": time.monotonic() - start,
                "controlwords": words,
            }
        )
        print(json.dumps(results[-1]), flush=True)
        traffic.close()
        writer.close()
        node.context.try_shutdown()
        with (options.output / "route.csv").open("w") as stream:
            output = csv.writer(stream)
            output.writerow(
                ("topic", "simulation_time", "x", "y", "linear", "angular")
            )
            output.writerows(positions)
        (options.output / "results.json").write_text(
            json.dumps(
                {
                    "seed": options.seed,
                    "domain": options.domain,
                    "channel": options.channel,
                    "left_node": numbers[0],
                    "right_node": numbers[1],
                    "checks": results,
                    "states": states,
                    "counts": counts,
                    "emergencies": emergencies,
                    "wall_duration": time.monotonic() - start,
                },
                indent=2,
            )
            + "\n"
        )
if not all(result["passed"] for result in results):
    raise SystemExit("CANopen acceptance did not pass every check")
print(options.output)
