"""Tests for the travel that the wheel encoders reckon reading by reading."""

import math
import secrets

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.rolling import Rolling


def test_rolling_drives_straight_by_the_rim_distance() -> None:
    """Equal counts on both wheels move the robot by their rim distance."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    resolution = secrets.SystemRandom().randint(1024, 8192)
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Rolling(
            Differential(radius, 0.288),
            resolution,
            (0.0, 0.0, 0.0),
            ((0, 0, 1.0), (0, 0, 1.02)),
        )
        .read(count, count, 1.04)
        .position()[0],
        close_to(count * 2 * math.pi / resolution * radius, 1e-9),
        "straight counts did not move the robot by the rim distance",
    )


def test_rolling_stays_in_place_when_spinning() -> None:
    """Opposite counts turn the robot without moving it."""
    count = secrets.SystemRandom().randint(1, 50000)
    assert_that(
        Rolling(
            Differential(0.033, 0.288),
            4096,
            (0.0, 0.0, 0.0),
            ((0, 0, 1.0), (0, 0, 1.02)),
        )
        .read(-count, count, 1.04)
        .position(),
        equal_to((0.0, 0.0)),
        "spinning counts did not keep the robot in place",
    )


def test_rolling_turns_by_the_encoders() -> None:
    """Opposite counts turn the heading by the turn the wheels make."""
    track = secrets.SystemRandom().uniform(0.1, 0.5)
    count = secrets.SystemRandom().randint(1, 2000)
    assert_that(
        Rolling(
            Differential(0.033, track),
            4096,
            (0.0, 0.0, 0.0),
            ((0, 0, 1.0), (0, 0, 1.02)),
        )
        .read(-count, count, 1.04)
        .heading(),
        close_to(0.033 * 2 * count * 2 * math.pi / 4096 / track, 1e-9),
        "opposite counts did not turn the heading by the wheels",
    )


def test_rolling_measures_the_forward_speed() -> None:
    """Equal counts over the interval give the rim speed forward."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    count = secrets.SystemRandom().randint(1, 5000)
    interval = secrets.SystemRandom().uniform(0.001, 0.1)
    assert_that(
        Rolling(
            Differential(radius, 0.288),
            4096,
            (0.7, -0.2, 1.1),
            ((-40, 25, 4.0), (0, 0, 5.0)),
        )
        .read(count, count, 5.0 + interval)
        .velocity()[0],
        close_to(count * 2 * math.pi / 4096 * radius / interval, 1e-6),
        "equal counts did not give the rim speed forward",
    )


def test_rolling_measures_the_yaw_rate() -> None:
    """Opposite counts over the interval give the turn per second."""
    track = secrets.SystemRandom().uniform(0.1, 0.5)
    count = secrets.SystemRandom().randint(1, 2000)
    interval = secrets.SystemRandom().uniform(0.001, 0.1)
    assert_that(
        Rolling(
            Differential(0.033, track),
            4096,
            (0.7, -0.2, 1.1),
            ((90, -12, 4.0), (0, 0, 5.0)),
        )
        .read(-count, count, 5.0 + interval)
        .velocity()[1],
        close_to(
            0.033 * 2 * count * 2 * math.pi / 4096 / track / interval, 1e-6
        ),
        "opposite counts did not give the turn per second",
    )


def test_rolling_measures_only_the_last_interval() -> None:
    """Older readings drop out, so the speed follows the last two only."""
    count = secrets.SystemRandom().randint(1, 5000)
    assert_that(
        Rolling(
            Differential(0.033, 0.288),
            4096,
            (0.0, 0.0, 0.0),
            ((0, 0, 1.0), (count, count, 2.0)),
        )
        .read(count, count, 3.0)
        .velocity(),
        equal_to((0.0, 0.0)),
        "travel did not measure the last interval alone",
    )
