"""Tests for the turn before the first yaw rate."""

import math
import secrets

from hamcrest import assert_that, close_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.gyroscopic import Gyroscopic
from robocell_sim.robot.implementations.odometry.still import Still


def test_still_turns_the_odometry_by_its_angle() -> None:
    """Before any rate, the turn is the angle it started with."""
    angle = secrets.SystemRandom().uniform(-math.pi, math.pi)
    assert_that(
        Still(angle)
        .apply(
            Gyroscopic(
                Differential(0.033, 0.288), 4096, (0, 0, 0.0), (0, 0, 0.0)
            ),
            0,
            0,
        )
        .heading(),
        close_to(angle, 1e-12),
        "odometry did not turn by the angle the turn started with",
    )


def test_still_adds_nothing_for_the_first_rate() -> None:
    """The first rate only starts the clock, since no time passed before it."""
    angle = secrets.SystemRandom().uniform(-math.pi, math.pi)
    assert_that(
        Still(angle)
        .read(
            secrets.SystemRandom().uniform(-3.0, 3.0),
            secrets.SystemRandom().uniform(0.0, 900.0),
        )
        .apply(
            Gyroscopic(
                Differential(0.033, 0.288), 4096, (0, 0, 0.0), (0, 0, 0.0)
            ),
            0,
            0,
        )
        .heading(),
        close_to(angle, 1e-12),
        "first rate added to the turn although no time passed",
    )
