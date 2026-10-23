"""Tests for the dots that mark lidar echoes in the viewer."""

import math
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import assert_that, equal_to, less_than
from robocell_sim.display.implementations.dots import Dots
from robocell_sim.display.implementations.scanned import Scanned


def test_dots_mark_every_echo() -> None:
    """Painting adds one dot per echo to the scene."""
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
    Dots(Scanned(data, "base_scan", scan)).paint(scene)
    assert_that(
        scene.ngeom,
        equal_to(sum(1 for distance in scan if math.isfinite(distance))),
        "scene did not receive one dot per echo",
    )


def test_dots_sit_where_the_rays_echo() -> None:
    """Each dot sits at the point where its ray echoed."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    echoes = Scanned(
        data,
        "base_scan",
        tuple(secrets.SystemRandom().uniform(0.12, 3.5) for _ in range(360)),
    )
    scene = mujoco.MjvScene(model, 1000)
    Dots(echoes).paint(scene)
    assert_that(
        float(
            np.abs(
                np.array([scene.geoms[i].pos for i in range(scene.ngeom)])
                - np.array(echoes.points())
            ).max()
        ),
        less_than(1e-5),
        "dots did not sit where the rays echoed",
    )


def test_dots_stop_at_the_capacity_of_the_scene() -> None:
    """A scene too small for every echo fills up without overflowing."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    capacity = secrets.SystemRandom().randint(1, 100)
    scene = mujoco.MjvScene(model, capacity)
    Dots(Scanned(data, "base_scan", (1.0,) * 360)).paint(scene)
    assert_that(
        scene.ngeom,
        equal_to(capacity),
        "a small scene did not fill up to its capacity",
    )
