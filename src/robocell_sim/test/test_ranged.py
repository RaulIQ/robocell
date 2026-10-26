"""Tests for the report that publishes the scan of the lidar."""

import math
import secrets
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.raycast import Raycast
from robocell_sim.ros.implementations.ranged import Ranged

from test.echoed import Echoed
from test.kept import Kept


def test_ranged_publishes_the_ranges_the_lidar_reads() -> None:
    """The scan carries every range of the lidar, no echo included."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    limits = (0.12, secrets.SystemRandom().uniform(1.0, 3.5))
    messages: list[Any] = []
    Ranged(
        data, Raycast(model, data, "base_scan", limits), limits, Kept(messages)
    ).publish()
    assert_that(
        tuple(float(echo) for echo in messages[-1].ranges),
        equal_to(
            tuple(
                float(np.float32(echo))
                for echo in Raycast(model, data, "base_scan", limits).ranges()
            )
        ),
        "scan did not carry the ranges the lidar read",
    )


def test_ranged_spreads_the_rays_over_a_full_turn() -> None:
    """The rays split the full turn evenly, however many there are."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    count = secrets.SystemRandom().randint(90, 720)
    messages: list[Any] = []
    Ranged(
        mujoco.MjData(model),
        Echoed(math.inf, count),
        (0.12, 3.5),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].angle_increment,
        close_to(2 * math.pi / count, 1e-6),
        "scan did not split the full turn evenly between the rays",
    )


def test_ranged_ends_the_scan_one_ray_short_of_a_full_turn() -> None:
    """The last ray points one increment before the first one again."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    count = secrets.SystemRandom().randint(90, 720)
    messages: list[Any] = []
    Ranged(
        mujoco.MjData(model),
        Echoed(math.inf, count),
        (0.12, 3.5),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].angle_max - messages[-1].angle_min,
        close_to(2 * math.pi * (count - 1) / count, 1e-5),
        "scan did not end one ray short of a full turn",
    )


def test_ranged_reports_the_limits_of_the_lidar() -> None:
    """The scan names the shortest and the longest range it can hold."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    limits = (
        secrets.SystemRandom().uniform(0.05, 0.5),
        secrets.SystemRandom().uniform(2.0, 12.0),
    )
    messages: list[Any] = []
    Ranged(
        mujoco.MjData(model), Echoed(math.inf, 360), limits, Kept(messages)
    ).publish()
    assert_that(
        math.dist((messages[-1].range_min, messages[-1].range_max), limits),
        close_to(0.0, 1e-5),
        "scan did not report the limits of the lidar",
    )


def test_ranged_places_the_scan_in_the_lidar_frame() -> None:
    """The scan lies in base_scan, the lidar link of the URDF."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    messages: list[Any] = []
    Ranged(
        mujoco.MjData(model),
        Echoed(math.inf, 360),
        (0.12, 3.5),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].header.frame_id,
        equal_to("base_scan"),
        "scan did not lie in the frame of the lidar",
    )


def test_ranged_stamps_the_scan_with_the_simulated_time() -> None:
    """The scan carries the time of the simulation it publishes."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=secrets.SystemRandom().randint(2, 900))
    messages: list[Any] = []
    Ranged(data, Echoed(math.inf, 360), (0.12, 3.5), Kept(messages)).publish()
    assert_that(
        messages[-1].header.stamp.sec
        + messages[-1].header.stamp.nanosec * 1e-9,
        close_to(data.time, 1e-9),
        "scan did not carry the simulated time",
    )
