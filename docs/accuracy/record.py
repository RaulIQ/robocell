#!/usr/bin/env python3
"""Record reproducible headless MuJoCo, wheel odometry and EKF runs."""

import argparse
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np
import rclpy
import rosbag2_py
import yaml
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from rclpy.time import Time
from robocell_sim.hardware.implementations.imu.gaussian import Gaussian
from robocell_sim.hardware.implementations.imu.noisy import Noisy
from robocell_sim.hardware.implementations.imu.packed import Packed
from robocell_sim.hardware.implementations.imu.sensed import Sensed
from robocell_sim.hardware.implementations.wheels.simulated import Simulated
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.imu.received import Received
from robocell_sim.robot.implementations.imu.scaled import Scaled
from robocell_sim.robot.implementations.imu.silent import Silent
from robocell_sim.ros.implementations.imu.inertial import Inertial
from robocell_sim.ros.implementations.pose.tracked import Tracked
from robocell_sim.ros.implementations.wheels.jointed import Jointed
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

parser = argparse.ArgumentParser()
parser.add_argument("output", type=Path)
parser.add_argument("--seed", type=int, default=121)
parser.add_argument("--domain", type=int, default=91)
parser.add_argument("--linear", type=float, default=7.1e-6)
parser.add_argument("--angular", type=float, default=7.9e-5)
parser.add_argument("--validation", action="store_true")
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
root = Path(__file__).resolve().parents[2]
model = mujoco.MjModel.from_xml_path(
    str(root / "src/robocell_sim/models/waffle_pi/workshop.xml")
)
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
mujoco.mj_forward(model, data)
rng = np.random.default_rng(args.seed)
bias = tuple(float(x) for x in rng.normal(0, 0.0075, 3))
imu = Noisy(
    Sensed(data, "imu_gyro", "imu_accelerometer"),
    Gaussian(bias, 0.001, rng),
    Gaussian((0.0, 0.0, 0.0), 0.02, rng),
)
drive = Differential(0.033, 0.288)
route = [
    (2, 0, 0),
    (4, 0, -0.4),
    (8, 0.12, 0),
    (1, 0, 0),
    (4, 0, 0.4),
    (8, 0.12, 0),
    (4, -0.08, 0),
    (2, 0, 0),
]
if args.validation:
    route = [
        (2, 0, 0),
        (4, 0, -0.4),
        (7, 0.12, 0),
        (1, 0, 0),
        (3.8, 0, 0.4),
        (5, 0.10, 0),
        (4, 0.08, -0.25),
        (4, -0.07, 0),
        (2, 0, 0),
    ]
ends = np.cumsum([segment[0] for segment in route])
writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(
        uri=str(args.output / "bag"), storage_id="sqlite3"
    ),
    rosbag2_py.ConverterOptions("", ""),
)
types = {
    "/clock": ("rosgraph_msgs/msg/Clock", Clock),
    "/ground_truth/odom": ("nav_msgs/msg/Odometry", Odometry),
    "/joint_states": ("sensor_msgs/msg/JointState", JointState),
    "/imu": ("sensor_msgs/msg/Imu", Imu),
    "/wheel/odometry": ("nav_msgs/msg/Odometry", Odometry),
    "/odometry/filtered": ("nav_msgs/msg/Odometry", Odometry),
    "/tf": ("tf2_msgs/msg/TFMessage", TFMessage),
    "/tf_static": ("tf2_msgs/msg/TFMessage", TFMessage),
}
for index, (topic, (typename, _)) in enumerate(types.items()):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=index, name=topic, type=typename, serialization_format="cdr"
        )
    )
rclpy.init(domain_id=args.domain)
node = rclpy.create_node("accuracy")
publishers = {
    topic: node.create_publisher(cls, topic, 10)
    for topic, (_, cls) in types.items()
    if topic
    not in {"/wheel/odometry", "/odometry/filtered", "/tf", "/tf_static"}
}
subscriptions = []
for topic in (
    "/wheel/odometry",
    "/odometry/filtered",
    "/tf",
    "/imu",
    "/joint_states",
    "/ground_truth/odom",
):
    subscriptions.append(
        node.create_subscription(
            types[topic][1],
            topic,
            lambda message, name=topic: writer.write(
                name, serialize_message(message), round(data.time * 1e9)
            ),
            100,
        )
    )
static = node.create_publisher(
    TFMessage,
    "/tf_static",
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
transforms = []
for parent, child, position, yaw in [
    ("world", "odom", (-2.1, -2.5, 0.0), math.pi / 2),
    ("base_footprint", "base_link", (0.0, 0.0, 0.01), 0),
    ("base_link", "imu_link", (0.0, 0.0, 0.068), 0),
]:
    transforms.append(
        TransformStamped(
            header=Header(frame_id=parent),
            child_frame_id=child,
            transform=Transform(
                translation=Vector3(
                    x=position[0], y=position[1], z=position[2]
                ),
                rotation=Quaternion(z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
            ),
        )
    )
static.publish(TFMessage(transforms=transforms))
writer.write(
    "/tf_static", serialize_message(TFMessage(transforms=transforms)), 0
)
parameters = yaml.safe_load(
    (root / "src/robocell_sim/config/ekf.yaml").read_text()
)
(args.output / "ekf.yaml").write_text(yaml.safe_dump(parameters))
processes = []
logs = []
environment = {
    **os.environ,
    "ROS_DOMAIN_ID": str(args.domain),
    "PYTHONPATH": str(root / "src/robocell_sim")
    + ":"
    + os.environ.get("PYTHONPATH", ""),
}
commands = [
    [
        "python",
        str(root / "src/robocell_sim/robocell_sim/odometry.py"),
        "--ros-args",
        "-p",
        "use_sim_time:=true",
        "-p",
        f"linear_variance:={args.linear}",
        "-p",
        f"angular_variance:={args.angular}",
    ],
    [
        "/opt/ros/jazzy/lib/robot_localization/ekf_node",
        "--ros-args",
        "-r",
        "__node:=ekf_filter_node",
        "--params-file",
        str(args.output / "ekf.yaml"),
    ],
]
try:
    for index, command in enumerate(commands):
        stream = (args.output / f"process-{index}.log").open("w")
        logs.append(stream)
        processes.append(
            subprocess.Popen(
                command,
                env=environment,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        )
    deadline = time.monotonic() + 8
    while (
        publishers["/joint_states"].get_subscription_count() < 2
        or publishers["/imu"].get_subscription_count() < 2
    ) and time.monotonic() < deadline:
        publishers["/clock"].publish(Clock(clock=Time(nanoseconds=1).to_msg()))
        static.publish(TFMessage(transforms=transforms))
        rclpy.spin_once(node, timeout_sec=0.01)
    if (
        publishers["/joint_states"].get_subscription_count() < 2
        or publishers["/imu"].get_subscription_count() < 2
    ):
        raise TimeoutError(
            "ROS subscribers were not discovered within eight seconds"
        )
    wheels = {
        "wheel_left_joint": Simulated(data, "wheel_left_encoder", 4096),
        "wheel_right_joint": Simulated(data, "wheel_right_encoder", 4096),
    }
    pairing = Silent(0)
    start = time.monotonic()
    count = 0
    contacts = set()
    while data.time < ends[-1]:
        segment = route[
            min(
                int(np.searchsorted(ends, data.time, side="right")),
                len(route) - 1,
            )
        ]
        data.ctrl[:] = drive.wheels(segment[1], segment[2])
        mujoco.mj_step(model, data)
        count += 1
        for contact in data.contact:
            for geom in (contact.geom1, contact.geom2):
                if (
                    model.geom_bodyid[geom] == 0
                    and model.geom(geom).name != "floor"
                ):
                    contacts.add(model.geom(geom).name)
        stamp = Time(nanoseconds=round(data.time * 1e9)).to_msg()
        if count % 5 == 0:
            clock = Clock(clock=stamp)
            publishers["/clock"].publish(clock)
            writer.write(
                "/clock", serialize_message(clock), round(data.time * 1e9)
            )
            for identifier, payload in Packed(
                imu.rate(), imu.acceleration(), count // 5 % 256
            ).frames():
                pairing = pairing.read(Received(identifier, payload))
            Inertial(
                Scaled(pairing),
                Header(stamp=stamp, frame_id="imu_link"),
                (0.001**2, 0.02**2),
                publishers["/imu"],
            ).publish()
        if count % 10 == 0:
            Jointed(data, wheels, publishers["/joint_states"]).publish()
            Tracked(
                data, "base_link", publishers["/ground_truth/odom"]
            ).publish()
        rclpy.spin_once(node, timeout_sec=0)
        remaining = start + data.time - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    (args.output / "profile.json").write_text(
        json.dumps(
            {
                "seed": args.seed,
                "bias": bias,
                "route": route,
                "linear_variance": args.linear,
                "angular_variance": args.angular,
                "contacts": sorted(contacts),
                "duration": float(data.time),
                "elapsed": time.monotonic() - start,
            },
            indent=2,
        )
    )
finally:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
    for stream in logs:
        stream.close()
    node.destroy_node()
    rclpy.shutdown()
    writer.close()
print(args.output)
