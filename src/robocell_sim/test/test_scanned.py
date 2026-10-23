"""Tests for the echoes of a lidar scan in world axes."""

import math
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.implementations.raycast import Raycast
from robocell_sim.implementations.scanned import Scanned


def test_scanned_places_the_first_echo_on_the_wall_ahead() -> None:
    """The echo of the first ray lies on the face of the wall ahead."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Scanned(
            data,
            "base_scan",
            Raycast(model, data, "base_scan", (0.12, 3.5)).ranges(),
        ).points()[0][0],
        close_to(0.95, 1e-6),
        "first echo did not lie on the face of the wall ahead",
    )


def test_scanned_turns_the_echoes_with_the_robot() -> None:
    """After a quarter turn left the first echo lies on the wall to the left."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mju_axisAngle2Quat(
        data.joint("base").qpos[3:], np.array([0.0, 0.0, 1.0]), math.pi / 2
    )
    mujoco.mj_forward(model, data)
    assert_that(
        Scanned(
            data,
            "base_scan",
            Raycast(model, data, "base_scan", (0.12, 3.5)).ranges(),
        ).points()[0][1],
        close_to(1.45, 1e-6),
        "first echo did not turn to the wall on the left",
    )


def test_scanned_skips_rays_without_an_echo() -> None:
    """Rays that read infinity leave no point behind."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    scan = tuple(
        secrets.SystemRandom().uniform(0.12, 3.5)
        if secrets.choice((True, False))
        else math.inf
        for _ in range(360)
    )
    assert_that(
        len(Scanned(data, "base_scan", scan).points()),
        equal_to(sum(1 for distance in scan if math.isfinite(distance))),
        "rays without an echo did not stay out of the points",
    )


def test_scanned_starts_the_rays_at_the_scanner() -> None:
    """The rays start where the scanner body sits in the world."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    data.joint("base").qpos[:2] = (
        secrets.SystemRandom().uniform(-1, 1),
        secrets.SystemRandom().uniform(-1, 1),
    )
    mujoco.mj_forward(model, data)
    assert_that(
        Scanned(data, "base_scan", (math.inf,) * 360).origin().tolist(),
        equal_to(data.body("base_scan").xpos.tolist()),
        "rays did not start at the scanner",
    )
