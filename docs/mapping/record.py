#!/usr/bin/env python3
"""Record a bounded mapping loop and reload its occupancy map through Nav2."""

import argparse
import contextlib
import hashlib
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import can
import rclpy
import rosbag2_py
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap
from PIL import Image
import numpy as np
from rcl_interfaces.msg import Log
from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("output", type=Path)
parser.add_argument("--domain", type=int, default=91)
parser.add_argument("--channel", default="vcan0")
parser.add_argument(
    "--route", type=Path, default=Path(__file__).with_name("route.json")
)
args = parser.parse_args()
args.output = args.output.resolve()
args.output.mkdir(parents=True, exist_ok=False)
route = json.loads(args.route.read_text())
(args.output / "route.json").write_text(json.dumps(route, indent=2))
with can.Bus(interface="socketcan", channel=args.channel) as bus:
    if bus.recv(0.3) is not None:
        raise RuntimeError("The selected CAN channel already carries traffic")
root = Path(__file__).resolve().parents[2]
config = yaml.safe_load(
    (root / "src/robocell_sim/config/slam.yaml").read_text()
)
config["slam_toolbox"]["ros__parameters"]["debug_logging"] = True
(args.output / "slam.yaml").write_text(yaml.safe_dump(config))
rclpy.init(domain_id=args.domain)
node = rclpy.create_node("mapping_verification")
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
    "/map": ("nav_msgs/msg/OccupancyGrid", OccupancyGrid),
    "/tf": ("tf2_msgs/msg/TFMessage", TFMessage),
    "/tf_static": ("tf2_msgs/msg/TFMessage", TFMessage),
    "/clock": ("rosgraph_msgs/msg/Clock", Clock),
    "/ground_truth/odom": ("nav_msgs/msg/Odometry", Odometry),
    "/wheel/odometry": ("nav_msgs/msg/Odometry", Odometry),
    "/odometry/filtered": ("nav_msgs/msg/Odometry", Odometry),
    "/pose": (
        "geometry_msgs/msg/PoseWithCovarianceStamped",
        PoseWithCovarianceStamped,
    ),
    "/slam_toolbox/graph_visualization": (
        "visualization_msgs/msg/MarkerArray",
        MarkerArray,
    ),
    "/rosout": ("rcl_interfaces/msg/Log", Log),
    "/cmd_vel": ("geometry_msgs/msg/Twist", Twist),
    "/motor_driver/state": ("std_msgs/msg/String", String),
    "/gyro/state": ("std_msgs/msg/String", String),
    "/verification/map": ("nav_msgs/msg/OccupancyGrid", OccupancyGrid),
}
latest = {}
counts = {name: 0 for name in types}
checks = []
segments = []
authorities = {}
processes = []
streams = []
started = time.monotonic()


def receive(topic, message):
    latest[topic] = message
    counts[topic] += 1
    writer.write(topic, serialize_message(message), time.time_ns())
    if topic in {"/tf", "/tf_static"}:
        for transform in message.transforms:
            authorities.setdefault(transform.child_frame_id, set()).add(
                transform.header.frame_id
            )


for index, (topic, (typename, cls)) in enumerate(types.items()):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=index, name=topic, type=typename, serialization_format="cdr"
        )
    )
    node.create_subscription(
        cls,
        topic,
        lambda message, name=topic: receive(name, message),
        qos_profile_action_status_default
        if topic
        in {
            "/map",
            "/tf_static",
            "/verification/map",
            "/motor_driver/state",
            "/gyro/state",
        }
        else qos_profile_sensor_data,
    )
command = node.create_publisher(Twist, "/cmd_vel", 10)
pause = node.create_client(SetBool, "/sim/pause")


def launch(name, command):
    stream = (args.output / f"{name}.log").open("w")
    streams.append(stream)
    process = subprocess.Popen(
        command,
        env={
            **os.environ,
            "ROS_DOMAIN_ID": str(args.domain),
            "MUJOCO_GL": "egl",
        },
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    processes.append(process)
    return process


def pump():
    if processes[0].poll() is not None:
        raise RuntimeError("Mapping stack exited during the recorded route")
    rclpy.spin_once(node, timeout_sec=0.005)


def until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        pump()
    if not predicate():
        raise TimeoutError("Mapping verification exceeded its bounded wait")


def call(client, request, timeout=8):
    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError("Mapping service did not appear within its deadline")
    future = client.call_async(request)
    until(future.done, timeout)
    return future.result()


def advance(duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        command.publish(Twist())
        pump()


def check(name, condition, **evidence):
    result = {"name": name, "passed": bool(condition), **evidence}
    checks.append(result)
    print(json.dumps(result), flush=True)
    if not condition:
        raise AssertionError(f"Mapping verification failed: {name}")


def clock():
    return Time.from_msg(latest["/clock"].clock).nanoseconds * 1e-9


def yaw(orientation):
    return math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y**2 + orientation.z**2),
    )


def position():
    pose = latest["/odometry/filtered"].pose.pose
    return pose.position.x, pose.position.y, yaw(pose.orientation)


def drive(goal, heading=False):
    beginning = clock()
    deadline = time.monotonic() + 100
    publication = 0.0
    while time.monotonic() < deadline:
        x, y, angle = position()
        distance = 0.0 if heading else math.dist((x, y), goal)
        error = math.remainder(
            (goal if heading else math.atan2(goal[1] - y, goal[0] - x)) - angle,
            math.tau,
        )
        if (heading and abs(error) < 0.025) or (
            not heading and distance < route["tolerance"]
        ):
            command.publish(Twist())
            advance(0.25)
            segments.append(
                {
                    "goal": goal,
                    "heading": heading,
                    "start": beginning,
                    "finish": clock(),
                    "odom": position(),
                }
            )
            print(json.dumps({"segment": segments[-1]}), flush=True)
            return
        if time.monotonic() >= publication:
            command.publish(
                Twist(
                    linear=Vector3(
                        x=0.0
                        if heading or abs(error) > 0.12
                        else min(route["linear"], 0.8 * distance)
                    ),
                    angular=Vector3(
                        z=max(
                            -route["angular"],
                            min(route["angular"], 1.2 * error),
                        )
                    ),
                )
            )
            publication = time.monotonic() + 0.05
        pump()
    raise TimeoutError(
        f"Mapping route did not reach goal {goal} from {position()}"
    )


subprocess.run(
    [
        "g++",
        "-std=c++17",
        "-I/opt/ros/jazzy/include",
        "-I/usr/include/eigen3",
        *[
            "-I" + str(path)
            for path in Path("/opt/ros/jazzy/include").iterdir()
            if path.is_dir()
        ],
        str(Path(__file__).with_name("export.cpp")),
        "-L/opt/ros/jazzy/lib",
        "-Wl,-rpath,/opt/ros/jazzy/lib",
        "-lkartoSlamToolbox",
        "-lboost_serialization",
        "-ltbb",
        "-o",
        str(args.output / "export"),
    ],
    check=True,
    timeout=30,
)

launch(
    "launch",
    [
        "ros2",
        "launch",
        "robocell_sim",
        "mapping.launch.xml",
        "headless:=true",
        "rviz:=false",
        "teleop:=false",
        f"seed:={route['seed']}",
        f"can_channel:={args.channel}",
        "left_node:=17",
        "right_node:=29",
        f"lidar_port:={args.output / 'lidar'}",
        f"slam_params_file:={args.output / 'slam.yaml'}",
    ],
)
try:
    until(
        lambda: (
            counts["/scan/slam"] >= 10
            and "/map" in latest
            and "/odometry/filtered" in latest
            and latest.get("/gyro/state", String()).data == "ready"
            and latest.get("/motor_driver/state", String()).data == "enabled"
        ),
        45,
    )
    check("bounded_startup", True, seconds=time.monotonic() - started)
    advance(2)
    for goal in route["waypoints"]:
        drive(tuple(goal))
    drive(route["heading"], heading=True)
    advance(3)
    check(
        "route_completed",
        len(segments) == len(route["waypoints"]) + 1,
        simulated_seconds=clock(),
        segments=len(segments),
    )
    check("pause_service", call(pause, SetBool.Request(data=True)).success)
    advance(0.8)
    frozen = clock()
    advance(0.8)
    check("clock_paused", clock() == frozen, clock=frozen)
    check("resume_service", call(pause, SetBool.Request(data=False)).success)
    count = counts["/scan/slam"]
    until(lambda: counts["/scan/slam"] > count + 5 and clock() > frozen, 10)
    check("mapping_resumed", True)
    advance(2)
    saver = launch(
        "save",
        [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            str(args.output / "workshop"),
            "--fmt",
            "pgm",
            "--mode",
            "trinary",
            "--free",
            "0.196",
            "--occ",
            "0.65",
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "save_map_timeout:=8.0",
        ],
    )
    until(lambda: saver.poll() is not None, 15)
    check(
        "map_saved",
        saver.returncode == 0
        and (args.output / "workshop.pgm").is_file()
        and (args.output / "workshop.yaml").is_file(),
    )
    graph = node.create_client(
        SerializePoseGraph, "/slam_toolbox/serialize_map"
    )
    check(
        "graph_saved_separately",
        call(
            graph,
            SerializePoseGraph.Request(filename=str(args.output / "graph")),
        ).result
        == 0,
    )
    exported = subprocess.run(
        [
            str(args.output / "export"),
            str(args.output / "graph.posegraph"),
            str(args.output / "graph.json"),
        ],
        capture_output=True,
        text=True,
        check=True,
        timeout=10,
    )
    (args.output / "graph-export.log").write_text(exported.stdout)
    graph = json.loads((args.output / "graph.json").read_text())
    links = [edge for edge in graph["edges"] if abs(edge[0] - edge[1]) >= 40]
    check(
        "nonlocal_loop_constraints",
        bool(links),
        nodes=len(graph["nodes"]),
        edges=len(graph["edges"]),
        links=links,
    )
    launch(
        "reload",
        [
            "ros2",
            "run",
            "nav2_map_server",
            "map_server",
            "--ros-args",
            "-r",
            "__node:=verification_map_server",
            "-p",
            f"yaml_filename:={args.output / 'workshop.yaml'}",
            "-p",
            "topic_name:=/verification/map",
            "-p",
            "use_sim_time:=true",
        ],
    )
    lifecycle = node.create_client(
        ChangeState, "/verification_map_server/change_state"
    )
    check(
        "map_server_configured",
        call(
            lifecycle, ChangeState.Request(transition=Transition(id=1))
        ).success,
    )
    check(
        "map_server_activated",
        call(
            lifecycle, ChangeState.Request(transition=Transition(id=3))
        ).success,
    )
    until(lambda: "/verification/map" in latest, 8)
    service = node.create_client(GetMap, "/verification_map_server/map")
    loaded = call(service, GetMap.Request()).map
    check(
        "map_server_reloaded",
        loaded.info.width == latest["/map"].info.width
        and loaded.info.height == latest["/map"].info.height
        and loaded.info.resolution == latest["/map"].info.resolution,
        width=loaded.info.width,
        height=loaded.info.height,
    )
    metadata = yaml.safe_load((args.output / "workshop.yaml").read_text())
    image = np.array(Image.open(args.output / metadata["image"]))
    expected = (
        np.flipud(np.where(image == 0, 100, np.where(image == 254, 0, -1)))
        .ravel()
        .tolist()
    )
    check(
        "reloaded_cells_match_saved_map",
        list(loaded.data) == expected,
        cells=len(expected),
    )
    check(
        "reloaded_origin_matches_saved_map",
        math.isclose(
            loaded.info.origin.position.x, metadata["origin"][0], abs_tol=1e-9
        )
        and math.isclose(
            loaded.info.origin.position.y, metadata["origin"][1], abs_tol=1e-9
        ),
        origin=metadata["origin"],
    )
    (args.output / "loaded.json").write_text(
        json.dumps(
            {
                "width": loaded.info.width,
                "height": loaded.info.height,
                "resolution": loaded.info.resolution,
                "data_sha256": hashlib.sha256(
                    bytes(value + 1 for value in loaded.data)
                ).hexdigest(),
            }
        )
    )
    check(
        "exclusive_tf_parents",
        authorities.get("odom") == {"map"}
        and authorities.get("base_footprint") == {"odom"}
        and authorities.get("base_link") == {"base_footprint"}
        and authorities.get("base_scan") == {"base_link"},
        parents={
            child: sorted(parents) for child, parents in authorities.items()
        },
    )
    check(
        "tf_publishers",
        {
            endpoint.node_name
            for endpoint in node.get_publishers_info_by_topic("/tf")
        }
        == {"slam_toolbox", "ekf_filter_node", "robot_state_publisher"},
        publishers=[
            endpoint.node_name
            for endpoint in node.get_publishers_info_by_topic("/tf")
        ],
    )
    check(
        "amcl_absent",
        not any(
            name == "amcl" for name, _ in node.get_node_names_and_namespaces()
        ),
    )
finally:
    command.publish(Twist())
    for process in reversed(processes):
        if process.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2)
    writer.close()
    node.context.try_shutdown()
    for stream in streams:
        stream.close()
    (args.output / "checks.json").write_text(
        json.dumps(
            {
                "checks": checks,
                "counts": counts,
                "segments": segments,
                "duration": time.monotonic() - started,
                "bag_sha256": hashlib.sha256(
                    (args.output / "bag/bag_0.db3").read_bytes()
                ).hexdigest(),
            },
            indent=2,
        )
    )

if not all(result["passed"] for result in checks):
    raise AssertionError("Mapping recording contains failed checks")
