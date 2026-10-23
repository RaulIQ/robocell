"""Tests for the lidar points placed in room axes by a pose."""

import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.raycast import Raycast
from robocell_sim.robot.implementations.placed import Placed


def test_placed_puts_the_echo_ahead_along_the_heading() -> None:
    """The first ray lands ahead of the scanner along the heading."""
    heading = secrets.SystemRandom().uniform(-math.pi, math.pi)
    distance = secrets.SystemRandom().uniform(0.12, 3.5)
    assert_that(
        Placed(
            (distance,) + (math.inf,) * 359, (0.0, 0.0, heading), -0.064
        ).points()[0][1],
        close_to((distance - 0.064) * math.sin(heading), 1e-9),
        "echo ahead did not land along the heading",
    )


def test_placed_puts_the_ninetieth_echo_to_the_left() -> None:
    """Index 90 lands to the left of a robot facing east."""
    distance = secrets.SystemRandom().uniform(0.12, 3.5)
    scan = [math.inf] * 360
    scan[90] = distance
    assert_that(
        Placed(tuple(scan), (0.0, 0.0, 0.0), -0.064).points()[0][1],
        close_to(distance, 1e-9),
        "ninetieth echo did not land to the left",
    )


def test_placed_moves_the_points_with_the_robot() -> None:
    """Shifting the pose shifts every point by the same offset."""
    x = secrets.SystemRandom().uniform(-3.0, 3.0)
    scan = (1.0,) + (math.inf,) * 359
    assert_that(
        Placed(scan, (x, 0.0, 0.0), -0.064).points()[0][0],
        close_to(x + 1.0 - 0.064, 1e-9),
        "points did not move with the robot",
    )


def test_placed_leaves_out_rays_without_an_echo() -> None:
    """Rays that read infinity leave no point on the map."""
    scan = tuple(
        secrets.SystemRandom().uniform(0.12, 3.5)
        if secrets.choice((True, False))
        else math.inf
        for _ in range(360)
    )
    assert_that(
        len(Placed(scan, (0.0, 0.0, 0.0), -0.064).points()),
        equal_to(sum(1 for distance in scan if math.isfinite(distance))),
        "rays without an echo did not stay off the map",
    )


def test_placed_maps_the_wall_ahead_where_it_stands() -> None:
    """A real scan placed by the true pose puts the wall at its face."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Placed(
            Raycast(model, data, "base_scan", (0.12, 3.5)).ranges(),
            (0.0, 0.0, 0.0),
            -0.064,
        ).points()[0][0],
        close_to(0.95, 1e-6),
        "wall ahead did not land at its face",
    )
