"""Tests for the lines that draw lidar rays in the viewer."""

import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.display.implementations.lidar.beams import Beams
from robocell_sim.display.implementations.lidar.scanned import Scanned


def test_beams_draw_a_line_per_echo() -> None:
    """Painting adds one line per echo to the scene."""
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
    scene = mujoco.MjvScene(model, 1000)
    Beams(Scanned(data, "base_scan", scan)).paint(scene)
    assert_that(
        [scene.geoms[index].type for index in range(scene.ngeom)],
        equal_to(
            [mujoco.mjtGeom.mjGEOM_LINE]
            * sum(1 for distance in scan if math.isfinite(distance))
        ),
        "scene did not receive one line per echo",
    )


def test_beams_reach_from_the_scanner_to_the_echo() -> None:
    """Each line is as long as the range its ray read."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    distance = secrets.SystemRandom().uniform(0.12, 3.5)
    scene = mujoco.MjvScene(model, 1000)
    Beams(Scanned(data, "base_scan", (distance,) * 360)).paint(scene)
    assert_that(
        float(scene.geoms[secrets.SystemRandom().randint(0, 359)].size[2]),
        close_to(distance, 1e-6),
        "line did not reach from the scanner to the echo",
    )


def test_beams_stop_at_the_capacity_of_the_scene() -> None:
    """A scene too small for every ray fills up without overflowing."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    capacity = secrets.SystemRandom().randint(1, 100)
    scene = mujoco.MjvScene(model, capacity)
    Beams(Scanned(data, "base_scan", (1.0,) * 360)).paint(scene)
    assert_that(
        scene.ngeom,
        equal_to(capacity),
        "a small scene did not fill up to its capacity",
    )
