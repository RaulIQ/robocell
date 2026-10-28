"""Tests for the travel that has read the wheel encoders once."""

import math
import secrets

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.started import Started


def test_started_reports_no_velocity() -> None:
    """One reading spans no interval, so the robot stands still."""
    assert_that(
        Started(
            Differential(0.033, 0.288),
            4096,
            (0.4, -1.2, 0.3),
            (
                secrets.SystemRandom().randint(-50000, 50000),
                secrets.SystemRandom().randint(-50000, 50000),
                secrets.SystemRandom().uniform(0.0, 1e4),
            ),
        ).velocity(),
        equal_to((0.0, 0.0)),
        "started travel did not stand still",
    )


def test_started_moves_by_the_counts_since_its_reading() -> None:
    """Equal counts since the first reading move the robot by their rim arc."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    origin = (
        secrets.SystemRandom().randint(-50000, 50000),
        secrets.SystemRandom().randint(-50000, 50000),
    )
    count = secrets.SystemRandom().randint(1, 5000)
    assert_that(
        Started(
            Differential(radius, 0.288),
            4096,
            (0.0, 0.0, 0.0),
            (origin[0], origin[1], 7.5),
        )
        .read(origin[0] + count, origin[1] + count, 7.52)
        .position()[0],
        close_to(count * 2 * math.pi / 4096 * radius, 1e-9),
        "started travel did not move by the counts since its reading",
    )


def test_started_times_the_next_reading_from_its_own() -> None:
    """The next reading measures its velocity against the first one."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    count = secrets.SystemRandom().randint(1, 5000)
    interval = secrets.SystemRandom().uniform(0.001, 0.1)
    assert_that(
        Started(Differential(radius, 0.288), 4096, (0.0, 0.0, 0.0), (0, 0, 3.0))
        .read(count, count, 3.0 + interval)
        .velocity()[0],
        close_to(count * 2 * math.pi / 4096 * radius / interval, 1e-6),
        "started travel did not time the next reading from its own",
    )
