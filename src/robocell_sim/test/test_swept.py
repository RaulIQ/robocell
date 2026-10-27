"""Tests for the turn that adds up yaw rates over time."""

import math
import secrets
from typing import TYPE_CHECKING

from hamcrest import assert_that, close_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.gyroscopic import Gyroscopic
from robocell_sim.robot.implementations.odometry.swept import Swept

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.turn import Turn


def test_swept_adds_the_mean_rate_times_the_time_between() -> None:
    """Two rates add their mean times the time between their stamps."""
    first = secrets.SystemRandom().uniform(-3.0, 3.0)
    second = secrets.SystemRandom().uniform(-3.0, 3.0)
    start = secrets.SystemRandom().uniform(0.0, 900.0)
    gap = secrets.SystemRandom().uniform(0.001, 0.05)
    assert_that(
        Swept(0.0, first, start)
        .read(second, start + gap)
        .apply(
            Gyroscopic(
                Differential(0.033, 0.288), 4096, (0, 0, 0.0), (0, 0, 0.0)
            ),
            0,
            0,
        )
        .heading(),
        close_to((first + second) / 2 * gap, 1e-9),
        "turn did not add the mean rate times the time between",
    )


def test_swept_adds_up_a_steady_rate_over_many_readings() -> None:
    """A steady rate over a second of readings turns by that rate."""
    rate = secrets.SystemRandom().uniform(-3.0, 3.0)
    turn: Turn = Swept(0.0, rate, 0.0)
    for count in range(1, 101):
        turn = turn.read(rate, count * 0.01)
    assert_that(
        turn.apply(
            Gyroscopic(
                Differential(0.033, 0.288), 4096, (0, 0, 0.0), (0, 0, 0.0)
            ),
            0,
            0,
        ).heading(),
        close_to(rate, 1e-9),
        "steady rate over a second did not turn by that rate",
    )


def test_swept_hands_the_wheel_counts_to_the_odometry() -> None:
    """The odometry moves by the wheels the turn is applied with."""
    ticks = secrets.SystemRandom().randint(1, 4096)
    assert_that(
        Swept(0.0, 0.0, 0.0)
        .apply(
            Gyroscopic(
                Differential(0.033, 0.288), 4096, (0, 0, 0.0), (0, 0, 0.0)
            ),
            ticks,
            ticks,
        )
        .position()[0],
        close_to(ticks * 2 * math.pi / 4096 * 0.033, 1e-9),
        "odometry did not move by the wheel counts it was given",
    )
