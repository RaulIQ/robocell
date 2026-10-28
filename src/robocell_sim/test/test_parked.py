"""Tests for the travel that has read no wheel encoder yet."""

import math
import secrets

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.parked import Parked


def test_parked_stands_where_the_robot_starts() -> None:
    """Before any reading the position is the one the robot starts at."""
    pose = (
        secrets.SystemRandom().uniform(-3.0, 3.0),
        secrets.SystemRandom().uniform(-3.0, 3.0),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    )
    assert_that(
        Parked(Differential(0.033, 0.288), 4096, pose).position(),
        equal_to(pose[:2]),
        "parked travel did not stand where the robot starts",
    )


def test_parked_faces_where_the_robot_starts() -> None:
    """Before any reading the heading is the one the robot starts with."""
    heading = secrets.SystemRandom().uniform(-math.pi, math.pi)
    assert_that(
        Parked(
            Differential(0.033, 0.288), 4096, (0.4, -1.2, heading)
        ).heading(),
        equal_to(heading),
        "parked travel did not face where the robot starts",
    )


def test_parked_reports_no_velocity() -> None:
    """Without two readings there is no interval, so the robot stands still."""
    assert_that(
        Parked(Differential(0.033, 0.288), 4096, (0.4, -1.2, 0.3)).velocity(),
        equal_to((0.0, 0.0)),
        "parked travel did not stand still",
    )


def test_parked_counts_from_the_first_reading() -> None:
    """The first counts only set the origin, so the robot does not leap."""
    assert_that(
        Parked(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0))
        .read(
            secrets.SystemRandom().randint(-50000, 50000),
            secrets.SystemRandom().randint(-50000, 50000),
            secrets.SystemRandom().uniform(0.0, 1e4),
        )
        .position()[0],
        close_to(0.0, 1e-12),
        "parked travel did not stay put on its first reading",
    )
