#!/usr/bin/env python3
"""Plot the recorded workshop route and compare wheel odometry and EKF."""

import argparse
import csv
import json
import math
import sqlite3
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock

arguments = argparse.ArgumentParser(description=__doc__)
arguments.add_argument("recording", type=Path)
options = arguments.parse_args()
with (options.recording / "route.csv").open() as stream:
    rows = list(csv.DictReader(stream))
series = {
    topic: np.array(
        [
            [
                float(row[key])
                for key in ("simulation_time", "x", "y", "linear", "angular")
            ]
            for row in rows
            if row["topic"] == topic
        ]
    )
    for topic in ("/ground_truth/odom", "/wheel/odometry", "/odometry/filtered")
}
truth = series["/ground_truth/odom"]
metrics = {}
fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
axes[0, 0].plot(
    truth[:, 1],
    truth[:, 2],
    color="black",
    label="MuJoCo ground truth",
    linewidth=2,
)
for topic, label, color in (
    ("/wheel/odometry", "CAN wheel odometry", "tab:blue"),
    ("/odometry/filtered", "EKF", "tab:orange"),
):
    samples = series[topic]
    x = -2.1 - samples[:, 2]
    y = -2.5 + samples[:, 1]
    error = np.hypot(
        x - np.interp(samples[:, 0], truth[:, 0], truth[:, 1]),
        y - np.interp(samples[:, 0], truth[:, 0], truth[:, 2]),
    )
    metrics[topic] = {
        "samples": len(samples),
        "rmse_mm": float(np.sqrt(np.mean(error**2)) * 1000),
        "maximum_mm": float(np.max(error) * 1000),
        "final_mm": float(error[-1] * 1000),
    }
    axes[0, 0].plot(x, y, label=label, color=color, linestyle="--")
    axes[1, 0].plot(samples[:, 0], error * 1000, label=label, color=color)
    axes[0, 1].plot(
        samples[:, 0], samples[:, 3], label=label, color=color, alpha=0.7
    )
    axes[1, 1].plot(
        samples[:, 0], samples[:, 4], label=label, color=color, alpha=0.7
    )
commands = []
clocks = []
with sqlite3.connect(options.recording / "bag/bag_0.db3") as connection:
    for topic, timestamp, payload in connection.execute(
        "SELECT topics.name, messages.timestamp, messages.data FROM messages JOIN topics ON messages.topic_id = topics.id WHERE topics.name IN ('/clock', '/cmd_vel', '/keyboard/cmd_vel') ORDER BY messages.timestamp"
    ):
        if topic == "/clock":
            message = deserialize_message(payload, Clock)
            clocks.append(
                (timestamp, message.clock.sec + message.clock.nanosec * 1e-9)
            )
        else:
            message = deserialize_message(
                payload, Twist if topic == "/cmd_vel" else TwistStamped
            )
            twist = message if topic == "/cmd_vel" else message.twist
            commands.append((timestamp, twist.linear.x, twist.angular.z))
clock = np.array(clocks)
command = np.array(commands)
index = np.clip(
    np.searchsorted(clock[:, 0], command[:, 0], side="right") - 1,
    0,
    len(clock) - 1,
)
axes[0, 1].step(
    clock[index, 1],
    command[:, 1],
    where="post",
    color="grey",
    linestyle=":",
    label="Raw operator request",
)
axes[1, 1].step(
    clock[index, 1],
    command[:, 2],
    where="post",
    color="grey",
    linestyle=":",
    label="Raw operator request",
)
axes[0, 1].plot(
    truth[:, 0],
    truth[:, 3],
    color="black",
    linewidth=1,
    label="MuJoCo measured",
)
axes[1, 1].plot(
    truth[:, 0],
    truth[:, 4],
    color="black",
    linewidth=1,
    label="MuJoCo measured",
)
axes[0, 0].scatter(
    [truth[0, 1]], [truth[0, 2]], color="green", marker="o", label="Start"
)
axes[0, 0].scatter(
    [truth[-1, 1]], [truth[-1, 2]], color="red", marker="x", label="Finish"
)
axes[0, 0].set(
    title="Workshop route through CANopen",
    xlabel="World x (m)",
    ylabel="World y (m)",
    aspect="equal",
)
axes[1, 0].set(
    title="Position error against ground truth",
    xlabel="Simulation time (s)",
    ylabel="Error (mm)",
)
axes[0, 1].set(
    title="Body translation and guard stops",
    xlabel="Simulation time (s)",
    ylabel="Forward velocity (m/s)",
)
axes[1, 1].set(
    title="Body rotation",
    xlabel="Simulation time (s)",
    ylabel="Yaw rate (rad/s)",
)
for axis in axes.flat:
    axis.grid(alpha=0.25)
    axis.legend(fontsize=8)
fig.savefig(options.recording / "route.png", dpi=160)
plt.close(fig)
metrics["truth_distance_m"] = float(
    np.sum(np.hypot(np.diff(truth[:, 1]), np.diff(truth[:, 2])))
)
metrics["simulation_duration_s"] = float(truth[-1, 0] - truth[0, 0])
metrics["coordinate_transform"] = (
    "World wheel/EKF position = (-2.1 - odom_y, -2.5 + odom_x)"
)
(options.recording / "metrics.json").write_text(
    json.dumps(metrics, indent=2) + "\n"
)
print(json.dumps(metrics, indent=2))
