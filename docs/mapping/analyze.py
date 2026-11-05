#!/usr/bin/env python3
"""Measure recorded SLAM poses, loop constraints and occupancy geometry."""

import argparse
import hashlib
import json
import math
import sqlite3
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
import yaml
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("recording", type=Path)
args = parser.parse_args()
connection = sqlite3.connect(args.recording / "bag/bag_0.db3")
topics = {
    id: (name, get_message(typename))
    for id, name, typename in connection.execute(
        "SELECT id,name,type FROM topics"
    )
}
records = {}
ages = []
clock = 0.0
motion_end = json.loads((args.recording / "checks.json").read_text())[
    "segments"
][-1]["finish"]
for topic, data in connection.execute(
    "SELECT topic_id,data FROM messages ORDER BY timestamp,id"
):
    name, cls = topics[topic]
    if name == "/clock":
        message = deserialize_message(data, cls)
        clock = message.clock.sec + message.clock.nanosec * 1e-9
    if name == "/scan" and clock <= motion_end:
        message = deserialize_message(data, cls)
        ages.append(
            clock
            - message.header.stamp.sec
            - message.header.stamp.nanosec * 1e-9
        )
    if name in {
        "/ground_truth/odom",
        "/odometry/filtered",
        "/pose",
        "/slam_toolbox/graph_visualization",
        "/scan",
        "/scan/slam",
    }:
        records.setdefault(name, []).append(deserialize_message(data, cls))
connection.close()


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def yaw(orientation):
    return math.atan2(
        2 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1 - 2 * (orientation.y**2 + orientation.z**2),
    )


def row(message):
    pose = message.pose.pose
    return (
        stamp(message),
        pose.position.x,
        pose.position.y,
        yaw(pose.orientation),
    )


truth = np.array([row(message) for message in records["/ground_truth/odom"]])
estimates = {
    name: np.array([row(message) for message in records[topic]])
    for name, topic in (("ekf", "/odometry/filtered"), ("slam", "/pose"))
}


def world(points):
    return np.column_stack((-2.1 - points[:, 1], -2.5 + points[:, 0]))


def residuals(rows):
    rows = rows[(rows[:, 0] >= truth[0, 0]) & (rows[:, 0] <= truth[-1, 0])]
    position = world(rows[:, 1:3])
    reference = np.column_stack(
        [
            np.interp(rows[:, 0], truth[:, 0], truth[:, column])
            for column in (1, 2)
        ]
    )
    angle = np.interp(rows[:, 0], truth[:, 0], np.unwrap(truth[:, 3]))
    return np.column_stack(
        (
            rows[:, 0],
            np.linalg.norm(position - reference, axis=1),
            np.arctan2(
                np.sin(rows[:, 3] + math.pi / 2 - angle),
                np.cos(rows[:, 3] + math.pi / 2 - angle),
            ),
        )
    )


def summary(rows):
    errors = residuals(rows)
    return {
        "samples": len(errors),
        "position_rmse_m": float(np.sqrt(np.mean(errors[:, 1] ** 2))),
        "position_p95_m": float(np.percentile(errors[:, 1], 95)),
        "position_max_m": float(max(errors[:, 1])),
        "position_final_m": float(errors[-1, 1]),
        "heading_rmse_deg": float(
            np.degrees(np.sqrt(np.mean(errors[:, 2] ** 2)))
        ),
        "heading_final_deg": float(np.degrees(errors[-1, 2])),
    }


def graph(message):
    nodes = {
        marker.id: (marker.pose.position.x, marker.pose.position.y)
        for marker in message.markers
        if marker.ns == "slam_toolbox" and marker.action == 0
    }
    edges = []
    for marker in message.markers:
        if marker.ns != "slam_toolbox_edges" or marker.id != 0:
            continue
        for earlier, later in zip(marker.points[::2], marker.points[1::2]):
            ends = tuple(
                min(
                    nodes,
                    key=lambda id: math.dist(nodes[id], (point.x, point.y)),
                )
                for point in (earlier, later)
            )
            if all(
                math.dist(nodes[id], (point.x, point.y)) < 1e-6
                for id, point in zip(ends, (earlier, later))
            ):
                edges.append(ends)
    return nodes, edges


constraints = []
revisions = []
seen = {}
for message in records.get("/slam_toolbox/graph_visualization", []):
    nodes, edges = graph(message)
    changed = [
        (id, math.dist(point, seen[id]))
        for id, point in nodes.items()
        if id in seen and math.dist(point, seen[id]) > 1e-6
    ]
    if changed:
        revisions.append(
            {
                "stamp": message.markers[-1].header.stamp.sec
                + message.markers[-1].header.stamp.nanosec * 1e-9,
                "nodes": len(changed),
                "maximum_m": max(distance for _, distance in changed),
                "ids": [id for id, _ in changed],
            }
        )
    seen.update(nodes)
    long = [edge for edge in edges if abs(edge[0] - edge[1]) >= 40]
    if long:
        constraints.append(
            {
                "stamp": message.markers[-1].header.stamp.sec
                + message.markers[-1].header.stamp.nanosec * 1e-9,
                "nodes": len(nodes),
                "edges": len(edges),
                "constraints": long,
            }
        )
saved = json.loads((args.recording / "graph.json").read_text())
nodes = {id: (x, y) for id, _, x, y, _ in saved["nodes"]}
edges = saved["edges"]
optimized = np.array(
    [(time, x, y, heading) for _, time, x, y, heading in saved["nodes"]]
)
online = {
    round(stamp(message) * 1e9): row(message) for message in records["/pose"]
}
corrections = [
    math.dist((x, y), online[round(time * 1e9)][1:3])
    for _, time, x, y, _ in saved["nodes"]
    if round(time * 1e9) in online
]
config = yaml.safe_load((args.recording / "workshop.yaml").read_text())
image = np.array(Image.open(args.recording / config["image"]))
indices = np.column_stack(np.where(image == 0))
points = np.column_stack(
    (
        config["origin"][0] + (indices[:, 1] + 0.5) * config["resolution"],
        config["origin"][1]
        + (image.shape[0] - indices[:, 0] - 0.5) * config["resolution"],
    )
)
points = world(points)
model = mujoco.MjModel.from_xml_path(
    str(
        Path(__file__).resolve().parents[2]
        / "src/robocell_sim/models/waffle_pi/workshop.xml"
    )
)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
distances = []
for geom in range(model.ngeom):
    kind = model.geom_type[geom]
    if model.geom_bodyid[geom] != 0 or (
        kind != mujoco.mjtGeom.mjGEOM_BOX
        and kind != mujoco.mjtGeom.mjGEOM_CYLINDER
    ):
        continue
    size = model.geom_size[geom]
    height = size[2] if kind == mujoco.mjtGeom.mjGEOM_BOX else size[1]
    if (
        not data.geom_xpos[geom, 2] - height
        <= 0.132
        <= data.geom_xpos[geom, 2] + height
    ):
        continue
    local = (
        np.column_stack((points, np.full(len(points), 0.132)))
        - data.geom_xpos[geom]
    ) @ data.geom_xmat[geom].reshape(3, 3)
    if kind == mujoco.mjtGeom.mjGEOM_BOX:
        delta = np.abs(local[:, :2]) - size[:2]
        distances.append(
            np.abs(
                np.linalg.norm(np.maximum(delta, 0), axis=1)
                + np.minimum(np.max(delta, axis=1), 0)
            )
        )
    else:
        distances.append(np.abs(np.linalg.norm(local[:, :2], axis=1) - size[0]))
surface = np.min(np.array(distances), axis=0)
result = {
    "scan_age": {
        "scope": "startup and driving, before map export",
        "minimum_s": min(ages),
        "maximum_s": max(ages),
        "samples": len(ages),
    },
    "alignment": {
        "world_x": "-2.1 - map_y",
        "world_y": "-2.5 + map_x",
        "world_yaw": "map_yaw + pi/2",
        "source": "fixed initial workshop pose, no trajectory fitting",
    },
    "ekf": summary(estimates["ekf"]),
    "slam_online": summary(estimates["slam"]),
    "optimized_graph": summary(optimized),
    "loop_graph": {
        "historical_pose_revisions": revisions,
        "final_correction_maximum_m": max(corrections) if corrections else None,
        "online_graph_stamp_matches": len(corrections),
        "nodes": len(nodes),
        "edges": len(edges),
        "long_constraint_snapshots": len(constraints),
        "first": constraints[0] if constraints else None,
        "last": constraints[-1] if constraints else None,
    },
    "route": {
        "truth_length_m": float(
            np.sum(np.linalg.norm(np.diff(truth[:, 1:3], axis=0), axis=1))
        ),
        "truth_return_distance_m": float(
            np.linalg.norm(truth[-1, 1:3] - truth[0, 1:3])
        ),
        "truth_return_heading_deg": float(
            np.degrees(math.remainder(truth[-1, 3] - truth[0, 3], math.tau))
        ),
    },
    "map": {
        "width": image.shape[1],
        "height": image.shape[0],
        "resolution": config["resolution"],
        "occupied_cells": len(points),
        "surface_median_m": float(np.median(surface)),
        "surface_p95_m": float(np.percentile(surface, 95)),
        "within_10cm_fraction": float(np.mean(surface < 0.1)),
        "image_sha256": hashlib.sha256(
            (args.recording / config["image"]).read_bytes()
        ).hexdigest(),
    },
    "limitations": [
        "Online SLAM poses are not retrospectively corrected after loop constraints",
        "Nearest-surface occupancy residuals do not measure unobserved map completeness",
        "Fixed initial alignment, no fitting to ground truth",
    ],
}
(args.recording / "constraints.json").write_text(
    json.dumps(constraints, indent=2)
)
valid = (
    bool(constraints)
    and bool(revisions)
    and len(corrections) == len(saved["nodes"])
)
result["validation"] = {
    "loop_constraints_and_optimization": valid,
    "raw_scans_remain_fresh": min(ages) >= -0.02 and max(ages) < 0.5,
    "pose_rmse_below_10cm": result["slam_online"]["position_rmse_m"] < 0.1,
    "map_surface_p95_below_15cm": result["map"]["surface_p95_m"] < 0.15,
}
(args.recording / "analysis.json").write_text(json.dumps(result, indent=2))
figure, axes = plt.subplots(1, 2, figsize=(12, 5))
axes[0].scatter(
    points[:, 0], points[:, 1], s=2, c="0.5", label="Occupied map cells"
)
axes[0].plot(truth[:, 1], truth[:, 2], color="black", label="Ground truth")
for name, rows in estimates.items():
    position = world(rows[:, 1:3])
    axes[0].plot(position[:, 0], position[:, 1], label=name.upper())
axes[0].set(
    xlabel="World x (m)",
    ylabel="World y (m)",
    title="Fixed initial alignment",
    aspect="equal",
)
axes[0].legend()
for name, rows in estimates.items():
    errors = residuals(rows)
    axes[1].plot(errors[:, 0], errors[:, 1], label=name.upper())
if revisions:
    axes[1].axvline(
        revisions[0]["stamp"],
        color="0.4",
        linestyle="--",
        label="First historical pose optimization",
    )
axes[1].set(
    xlabel="Simulation time (s)",
    ylabel="Position error (m)",
    title="Online pose errors",
)
axes[1].legend()
figure.tight_layout()
figure.savefig(args.recording / "comparison.png", dpi=160)
print(json.dumps(result, indent=2))

if not all(result["validation"].values()):
    raise AssertionError(
        "Recorded mapping did not pass loop and accuracy validation"
    )
