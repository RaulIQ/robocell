"""Tests for the odometry reckoned from wheel encoder counts."""

import math
import secrets
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
from hamcrest import assert_that, close_to, equal_to, less_than
from robocell_sim.hardware.implementations.simulated import Simulated
from robocell_sim.robot.implementations.differential import Differential
from robocell_sim.robot.implementations.reckoned import Reckoned

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.odometry import Odometry


def test_reckoned_drives_straight_by_the_rim_distance() -> None:
    """Equal counts on both wheels move the robot by their rim distance."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    resolution = secrets.SystemRandom().randint(1024, 8192)
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Reckoned(
            Differential(radius, 0.288), resolution, (0.0, 0.0, 0.0), (0, 0)
        )
        .read(count, count, secrets.SystemRandom().uniform(-50.0, 50.0))
        .position()[0],
        close_to(count * 2 * math.pi / resolution * radius, 1e-9),
        "straight counts did not move the robot by the rim distance",
    )


def test_reckoned_stays_in_place_when_spinning() -> None:
    """Opposite counts turn the robot without moving it."""
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Reckoned(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0))
        .read(-count, count, secrets.SystemRandom().uniform(-50.0, 50.0))
        .position(),
        equal_to((0.0, 0.0)),
        "spinning counts did not keep the robot in place",
    )


def test_reckoned_moves_along_its_heading() -> None:
    """A straight run goes in the direction the robot faces."""
    heading = secrets.SystemRandom().uniform(-math.pi, math.pi)
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Reckoned(Differential(0.033, 0.288), 4096, (0.0, 0.0, heading), (0, 0))
        .read(count, count, secrets.SystemRandom().uniform(-50.0, 50.0))
        .position()[1],
        close_to(count * 2 * math.pi / 4096 * 0.033 * math.sin(heading), 1e-9),
        "straight counts did not move the robot along its heading",
    )


def test_reckoned_counts_from_the_last_reading() -> None:
    """Reading the same counts again leaves the position where it was."""
    left = secrets.SystemRandom().randint(-50000, 50000)
    right = secrets.SystemRandom().randint(-50000, 50000)
    odometry = Reckoned(
        Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0)
    ).read(left, right, secrets.SystemRandom().uniform(-50.0, 50.0))
    assert_that(
        odometry.read(
            left, right, secrets.SystemRandom().uniform(-50.0, 50.0)
        ).position(),
        equal_to(odometry.position()),
        "same counts did not leave the position where it was",
    )


def test_reckoned_follows_the_simulated_waffle_pi() -> None:
    """Encoder odometry of a driving robot ends within a centimeter of it."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    data.ctrl = (
        secrets.SystemRandom().uniform(1.0, 3.0),
        secrets.SystemRandom().uniform(4.0, 6.0),
    )
    odometry: Odometry = Reckoned(
        Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0)
    )
    for _ in range(1500):
        mujoco.mj_step(model, data)
        odometry = odometry.read(
            Simulated(data, "wheel_left_encoder", 4096).ticks(),
            Simulated(data, "wheel_right_encoder", 4096).ticks(),
            secrets.SystemRandom().uniform(-50.0, 50.0),
        )
    assert_that(
        math.dist(odometry.position(), data.body("base_link").xpos[:2]),
        less_than(0.01),
        "encoder odometry did not stay within a centimeter of the robot",
    )


def test_reckoned_turns_its_heading_by_the_encoders() -> None:
    """Opposite counts turn the heading by the angle the wheels imply."""
    start = secrets.SystemRandom().uniform(-math.pi, math.pi)
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Reckoned(Differential(0.033, 0.288), 4096, (0.0, 0.0, start), (0, 0))
        .read(-count, count, secrets.SystemRandom().uniform(-50.0, 50.0))
        .heading(),
        close_to(start + 0.033 * 2 * count * 2 * math.pi / 4096 / 0.288, 1e-9),
        "heading did not turn by the angle of the encoders",
    )
