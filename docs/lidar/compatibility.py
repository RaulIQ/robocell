#!/usr/bin/env python3
"""Record the actual A1-to-SLAM path through motion, delayed TF and pauses."""

import argparse
import bisect
import contextlib
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import time
from pathlib import Path

import can
import mujoco
import numpy as np
import rclpy
import rosbag2_py
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path)
parser.add_argument("--domain", type=int, default=85)
parser.add_argument("--channel", default="vcan0")
parser.add_argument(
    "--stationary",
    action="store_true",
    help="Verify available integration without commanding motion",
)
args = parser.parse_args()
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=False)
with can.Bus(interface="socketcan", channel=args.channel) as bus:
    if bus.recv(0.3) is not None:
        raise RuntimeError("The selected CAN channel already carries traffic")
rclpy.init(domain_id=args.domain)
node = rclpy.create_node("scan_compatibility_verification")
writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(
        uri=str(args.output / "bag"), storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions("", ""),
)
types = {
    "/scan": ("sensor_msgs/msg/LaserScan", LaserScan),
    "/scan/slam": ("sensor_msgs/msg/LaserScan", LaserScan),
    "/clock": ("rosgraph_msgs/msg/Clock", Clock),
    "/ground_truth/odom": ("nav_msgs/msg/Odometry", Odometry),
    "/odometry/filtered": ("nav_msgs/msg/Odometry", Odometry),
    "/tf": ("tf2_msgs/msg/TFMessage", TFMessage),
    "/tf_static": ("tf2_msgs/msg/TFMessage", TFMessage),
    "/map": ("nav_msgs/msg/OccupancyGrid", OccupancyGrid),
    "/pose": (
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        PoseWithCovarianceStamped,
    ),
    "/motor_driver/state": ("std_msgs/msg/String", String),
    "/gyro/state": ("std_msgs/msg/String", String),
    "/cmd_vel": ("geometry_msgs/msg/Twist", Twist),
}
latest = {}
raw = []
ages = []
poses = []
maps = []
adapted = []
truth = []
checks = []
phase = "startup"
process = None
suspended = -1
started = time.monotonic()
for index, (topic, (typename, cls)) in enumerate(types.items()):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=index, name=topic, type=typename, serialization_format="cdr"
        )
    )


def receive(topic, message):
    latest[topic] = message
    writer.write(topic, serialize_message(message), time.time_ns())
    if topic == "/scan":
        raw.append(message)
        if "/clock" in latest:
            ages.append((clock() - stamp(message)) * 1e-9)
    if topic == "/scan/slam":
        adapted.append((phase, message))
    if topic == "/pose":
        poses.append(message)
    if topic == "/map":
        maps.append(message)
    if topic == "/ground_truth/odom":
        truth.append(message)


for topic, (_, cls) in types.items():
    node.create_subscription(
        cls,
        topic,
        lambda message, name=topic: receive(name, message),
        qos_profile_action_status_default
        if topic in {"/tf_static", "/map", "/motor_driver/state", "/gyro/state"}
        else qos_profile_sensor_data,
    )
client = node.create_client(SetBool, "/sim/pause")
command = node.create_publisher(Twist, "/cmd_vel", 10)


def pump():
    if process.poll() is not None:
        raise RuntimeError(
            "Mapping launch exited during compatibility verification"
        )
    rclpy.spin_once(node, timeout_sec=0.005)


def until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        pump()
    if not predicate():
        raise TimeoutError(
            "Compatibility verification exceeded its bounded wait"
        )


def advance(duration, linear=0.0, angular=0.0):
    deadline = time.monotonic() + duration
    publication = 0.0
    while time.monotonic() < deadline:
        if time.monotonic() >= publication:
            command.publish(
                Twist(linear=Vector3(x=linear), angular=Vector3(z=angular))
            )
            publication = time.monotonic() + 0.05
        pump()


def pause(value):
    if not client.wait_for_service(timeout_sec=3):
        raise TimeoutError("Workshop pause service did not appear")
    future = client.call_async(SetBool.Request(data=value))
    until(future.done, 5)
    if not future.result().success:
        raise RuntimeError("Workshop rejected the pause request")


def stamp(message):
    return Time.from_msg(message.header.stamp).nanoseconds


def clock():
    return Time.from_msg(latest["/clock"].clock).nanoseconds


def check(name, passed, **evidence):
    result = {"name": name, "passed": bool(passed), **evidence}
    checks.append(result)
    print(json.dumps(result), flush=True)
    if not passed:
        raise AssertionError(f"Scan compatibility check failed: {name}")


def scene():
    model = mujoco.MjModel.from_xml_path(
        str(
            Path(__file__).resolve().parents[2]
            / "src/robocell_sim/models/waffle_pi/workshop.xml"
        )
    )
    data = mujoco.MjData(model)
    stamps = [stamp(message) for message in truth]
    errors = {"rest": [], "turn": [], "arc": []}
    for phase, scan in adapted:
        if (
            phase not in errors
            or not stamps
            or stamp(scan) < stamps[0]
            or stamp(scan) > stamps[-1]
        ):
            continue
        index = bisect.bisect_left(stamps, stamp(scan))
        pose = truth[min(index, len(truth) - 1)].pose.pose
        data.qpos[:7] = [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
        ]
        mujoco.mj_forward(model, data)
        body = data.body("base_scan")
        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            point = body.xpos + body.xmat.reshape(3, 3) @ np.array(
                [distance * math.cos(angle), distance * math.sin(angle), 0.0]
            )
            surfaces = []
            for geom in range(model.ngeom):
                if (
                    model.geom_bodyid[geom] != 0
                    or model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_BOX
                    and model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_CYLINDER
                ):
                    continue
                local = data.geom_xmat[geom].reshape(3, 3).T @ (
                    point - data.geom_xpos[geom]
                )
                size = model.geom_size[geom]
                if abs(local[2]) > (
                    size[2]
                    if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_BOX
                    else size[1]
                ):
                    continue
                if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_BOX:
                    delta = np.abs(local[:2]) - size[:2]
                    surfaces.append(
                        abs(
                            float(
                                np.linalg.norm(np.maximum(delta, 0))
                                + min(float(max(delta)), 0)
                            )
                        )
                    )
                else:
                    surfaces.append(abs(math.hypot(*local[:2]) - size[0]))
            if surfaces:
                errors[phase].append(min(surfaces))
    for phase, values in errors.items():
        if args.stationary and phase != "rest":
            continue
        check(
            f"asymmetric_scene_{phase}",
            len(values) > 1000 and float(np.median(values)) < 0.08,
            points=len(values),
            median_m=float(np.median(values)) if values else None,
            p95_m=float(np.percentile(values, 95)) if values else None,
        )


with (args.output / "launch.log").open("w") as stream:
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "robocell_sim",
            "mapping.launch.xml",
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
                len(adapted) >= 10
                and "/map" in latest
                and latest.get("/gyro/state", String()).data == "ready"
                and latest.get("/motor_driver/state", String()).data
                == "enabled"
            ),
            45,
        )
        check("bounded_startup", True, seconds=time.monotonic() - started)
        phase = "rest"
        advance(3)
        if not args.stationary:
            phase = "turn"
            advance(5, angular=0.3)
            phase = "arc"
            advance(4, linear=0.06, angular=-0.2)
            phase = "settle"
            advance(1)
            pose = truth[-1].pose.pose.orientation
            check(
                "robot_turned",
                abs(pose.z - truth[0].pose.pose.orientation.z) > 0.1,
                final_orientation=[pose.w, pose.x, pose.y, pose.z],
            )
        ekf = int(
            re.search(
                r"\[ekf_node-\d+\]: process started with pid \[(\d+)\]",
                (args.output / "launch.log").read_text(),
            ).group(1)
        )
        for delay in (0.2, 1.0):
            phase = f"tf_{delay}"
            beginning = len(adapted)
            suspended = ekf
            os.kill(ekf, signal.SIGSTOP)
            advance(delay)
            os.kill(ekf, signal.SIGCONT)
            suspended = -1
            until(lambda: len(adapted) >= beginning + 6, 8)
            check(f"tf_recovery_{delay}", True, scans=len(adapted) - beginning)
        for cycle in range(2):
            phase = f"pause_{cycle}"
            pause(True)
            advance(0.6)
            frozen = clock()
            beginning = len(adapted)
            advance(0.8)
            check(
                f"pause_{cycle}",
                clock() == frozen
                and all(
                    stamp(scan) <= frozen for _, scan in adapted[beginning:]
                ),
                drained=len(adapted) - beginning,
            )
            pause(False)
            beginning = len(adapted)
            until(
                lambda: (
                    len(adapted) >= beginning + 6
                    and stamp(adapted[-1][1]) > frozen
                ),
                10,
            )
            check(f"resume_{cycle}", True, scans=len(adapted) - beginning)
        phase = "finish"
        advance(1)
        check(
            "raw_scan_preserved",
            {len(scan.ranges) for scan in raw} == {381, 382}
            and all(
                scan.angle_increment < 0 and scan.time_increment > 0
                for scan in raw
            )
            and len({scan.angle_min for scan in raw}) > 1,
            counts=sorted({len(scan.ranges) for scan in raw}),
            scans=len(raw),
        )
        grids = {
            (
                len(scan.ranges),
                scan.angle_min,
                scan.angle_max,
                scan.angle_increment,
                scan.time_increment,
                scan.header.frame_id,
            )
            for _, scan in adapted
        }
        check(
            "fixed_snapshot_grid",
            len(grids) == 1
            and next(iter(grids))[0] == 360
            and next(iter(grids))[4] == 0.0,
            grids=[list(grid) for grid in grids],
            scans=len(adapted),
        )
        check(
            "monotonic_reference",
            all(
                stamp(after[1]) > stamp(before[1])
                for before, after in zip(adapted, adapted[1:])
            ),
        )
        check(
            "one_raw_publisher",
            [
                endpoint.node_name
                for endpoint in node.get_publishers_info_by_topic("/scan")
            ]
            == ["lidar"],
        )
        check(
            "slam_subscribed_to_adapter",
            "slam_toolbox"
            in [
                endpoint.node_name
                for endpoint in node.get_subscriptions_info_by_topic(
                    "/scan/slam"
                )
            ],
        )
        check(
            "map_received",
            "/map" in latest
            and any(value >= 0 for value in latest["/map"].data),
            width=latest["/map"].info.width,
            height=latest["/map"].info.height,
        )
        check(
            "fresh_raw_scans",
            bool(ages) and min(ages) >= -0.02 and max(ages) < 0.5,
            minimum_s=min(ages),
            maximum_s=max(ages),
            first_s=ages[0],
            last_s=ages[-1],
        )
        references = {stamp(scan) for _, scan in adapted}
        check(
            "slam_processed_successive_snapshots",
            len(poses) >= 4
            and all(stamp(pose) in references for pose in poses),
            poses=len(poses),
            matched_references=sum(stamp(pose) in references for pose in poses),
        )
        check("map_updated", len(maps) > 2, updates=len(maps))
        scene()
    finally:
        with contextlib.suppress(ProcessLookupError):
            if suspended > 0:
                os.kill(suspended, signal.SIGCONT)
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        writer.close()
        node.context.try_shutdown()
        log = (args.output / "launch.log").read_text()
        checks.append(
            {
                "name": "no_slam_count_rejections",
                "passed": "range readings, expected" not in log,
            }
        )

        (args.output / "checks.json").write_text(
            json.dumps(
                {
                    "checks": checks,
                    "raw_scans": len(raw),
                    "adapted_scans": len(adapted),
                    "duration": time.monotonic() - started,
                    "bag_sha256": hashlib.sha256(
                        (args.output / "bag/bag_0.db3").read_bytes()
                    ).hexdigest(),
                },
                indent=2,
            )
        )

if not all(result["passed"] for result in checks):
    raise AssertionError("Compatibility verification recorded a failed check")
