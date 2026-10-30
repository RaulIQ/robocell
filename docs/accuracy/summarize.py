#!/usr/bin/env python3
"""Export measured covariance evidence and before/after accuracy plots."""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("recordings", type=Path)
parser.add_argument("output", type=Path)
arguments = parser.parse_args()
plt.switch_backend("Agg")
figure, axes = plt.subplots(2, 2, figsize=(11, 7), sharex="col")
summary = {}
for column, route in enumerate(("train", "validation")):
    for phase, color in (("before", "tab:orange"), ("after", "tab:blue")):
        directory = arguments.recordings / f"{route}-free-{phase}"
        metrics = json.loads((directory / "metrics.json").read_text())
        summary[f"{route}-{phase}"] = {
            "profile": json.loads((directory / "profile.json").read_text()),
            "metrics": metrics,
            "recording": str(directory),
            "sha256": hashlib.sha256(
                (directory / "bag/bag_0.db3").read_bytes()
            ).hexdigest(),
        }
        values = np.loadtxt(
            directory / "filtered-errors.csv", delimiter=",", skiprows=1
        )
        axes[0, column].plot(
            values[:, 0], values[:, 1] * 1000, color=color, label=f"EKF {phase}"
        )
        axes[1, column].plot(
            values[:, 0], np.degrees(values[:, 2]), color=color
        )
    values = np.loadtxt(
        arguments.recordings / f"{route}-free-before/odometry-errors.csv",
        delimiter=",",
        skiprows=1,
    )
    axes[0, column].plot(
        values[:, 0], values[:, 1] * 1000, color="black", label="Wheels"
    )
    axes[1, column].plot(values[:, 0], np.degrees(values[:, 2]), color="black")
    axes[0, column].set_title(
        "Calibration route, seed 121"
        if column == 0
        else "Validation route, seed 422"
    )
    axes[0, column].set_ylabel("Position error [mm]")
    axes[1, column].set_ylabel("Signed heading error [degrees]")
    axes[1, column].set_xlabel("Simulation time [s]")
    axes[0, column].legend()
    for row in range(2):
        axes[row, column].grid(alpha=0.3)
figure.suptitle("Wheel odometry and EKF against MuJoCo ground truth")
figure.tight_layout()
arguments.output.mkdir(parents=True, exist_ok=True)
figure.savefig(arguments.output / "04-ekf.png", dpi=180)
(arguments.output / "04-ekf.json").write_text(
    json.dumps(summary, indent=2) + "\n"
)
