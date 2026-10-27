"""Tests for the launch file of the workshop."""

import math
from pathlib import Path

from defusedxml import ElementTree
from hamcrest import assert_that, close_to, equal_to


def test_workshop_launch_fixes_odom_in_the_world() -> None:
    """A static transform hangs odom from the world frame of MuJoCo."""
    assert_that(
        [
            node.get("args", "").split()[-4:]
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "static_transform_publisher"
        ],
        equal_to([["--frame-id", "world", "--child-frame-id", "odom"]]),
        "launch file did not hang odom from the world frame",
    )


def test_workshop_launch_starts_odom_where_the_robot_starts() -> None:
    """Odom sits at the start keyframe of the workshop, x, y and heading."""
    qpos = [
        float(value)
        for value in next(
            key.get("qpos", "")
            for key in ElementTree.parse(
                Path(__file__).parents[1] / "models/waffle_pi/workshop.xml"
            ).iter("key")
            if key.get("name") == "start"
        ).split()
    ]
    args = next(
        node.get("args", "").split()
        for node in ElementTree.parse(
            Path(__file__).parents[1] / "launch/workshop.launch.xml"
        ).iter("node")
        if node.get("exec") == "static_transform_publisher"
    )
    assert_that(
        math.dist(
            [
                float(args[args.index(flag) + 1])
                for flag in ("--x", "--y", "--yaw")
            ],
            (qpos[0], qpos[1], 2 * math.atan2(qpos[6], qpos[3])),
        ),
        close_to(0.0, 1e-6),
        "odom did not sit where the robot starts in the workshop",
    )


def test_workshop_launch_runs_the_odometry_on_simulated_time() -> None:
    """The odometry starts with the simulator and follows simulated time."""
    assert_that(
        [
            {
                param.get("name"): param.get("value")
                for param in node.iter("param")
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "odometry.py"
        ],
        equal_to([{"use_sim_time": "true"}]),
        "launch file did not run the odometry on simulated time",
    )


def test_workshop_launch_runs_the_imu_driver_on_simulated_time() -> None:
    """The IMU driver starts with the simulator and stamps simulated time."""
    assert_that(
        [
            {
                param.get("name"): param.get("value")
                for param in node.iter("param")
            }
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "imu.py"
        ],
        equal_to([{"use_sim_time": "true"}]),
        "launch file did not run the IMU driver on simulated time",
    )
