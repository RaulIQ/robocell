#!/usr/bin/env python3
"""Compare recorded odometry using message timestamps and recorded TF."""

import argparse
import json
import math
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from docs.accuracy.aligned import Aligned

parser = argparse.ArgumentParser()
parser.add_argument("recording", type=Path)
args = parser.parse_args()
connection = sqlite3.connect(args.recording / "bag/bag_0.db3")
topics = {
    index: (name, get_message(typename))
    for index, name, typename in connection.execute(
        "select id,name,type from topics"
    )
}
streams = {
    name: []
    for name in ["/ground_truth/odom", "/wheel/odometry", "/odometry/filtered"]
}
offset = None
for topic, payload in connection.execute(
    "select topic_id,data from messages order by timestamp,id"
):
    name, cls = topics[topic]
    if name not in streams and name != "/tf_static":
        continue
    message = deserialize_message(payload, cls)
    if name == "/tf_static":
        for transform in message.transforms:
            if (
                transform.header.frame_id == "world"
                and transform.child_frame_id == "odom"
            ):
                q = transform.transform.rotation
                offset = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    math.atan2(
                        2 * (q.w * q.z + q.x * q.y),
                        1 - 2 * (q.y * q.y + q.z * q.z),
                    ),
                )
        continue
    q = message.pose.pose.orientation
    p = message.pose.pose.position
    expected = "world" if name == "/ground_truth/odom" else "odom"
    if message.header.frame_id != expected:
        raise ValueError(
            f"{name} has an unexpected frame: {message.header.frame_id}"
        )
    streams[name].append(
        (
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            p.x,
            p.y,
            math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            ),
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        )
    )
if offset is None:
    raise ValueError("Recorded world-to-odom transform is missing")
for name, stream in streams.items():
    streams[name] = tuple(sorted({row[0]: row for row in stream}.values()))
results = {
    "offset": offset,
    "counts": {name: len(stream) for name, stream in streams.items()},
}
for name in ("/wheel/odometry", "/odometry/filtered"):
    rows = np.array(
        Aligned(streams["/ground_truth/odom"], streams[name], offset).rows()
    )
    rows = rows[rows[:, 0] >= 2.0]
    if len(rows) == 0:
        raise ValueError(f"{name} has no aligned measurements")
    np.savetxt(
        args.recording / (name.split("/")[-1] + "-errors.csv"),
        rows,
        delimiter=",",
        header="time,position_error,heading_error,linear_residual,angular_residual",
        comments="",
    )
    results[name] = {
        "samples": len(rows),
        "first_time": float(rows[0, 0]),
        "last_time": float(rows[-1, 0]),
        "position_rms": float(np.sqrt(np.mean(rows[:, 1] ** 2))),
        "position_final": float(rows[-1, 1]),
        "position_max": float(np.max(rows[:, 1])),
        "heading_rms": float(np.sqrt(np.mean(rows[:, 2] ** 2))),
        "heading_final": float(rows[-1, 2]),
        "linear_mean": float(np.mean(rows[:, 3])),
        "linear_variance": float(np.var(rows[:, 3])),
        "angular_mean": float(np.mean(rows[:, 4])),
        "angular_variance": float(np.var(rows[:, 4])),
    }
(args.recording / "metrics.json").write_text(json.dumps(results, indent=2))
print(json.dumps(results, indent=2))
