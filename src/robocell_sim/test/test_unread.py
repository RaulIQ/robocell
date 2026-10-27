"""Tests for the odometry before the first reading of the wheels."""

import math
import secrets

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.unread import Unread


def test_unread_stays_at_its_pose_after_the_first_reading() -> None:
    """The first counts only set the start, whatever they are."""
    pose = (
        secrets.SystemRandom().uniform(-3.0, 3.0),
        secrets.SystemRandom().uniform(-3.0, 3.0),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    )
    assert_that(
        Unread(Differential(0.033, 0.288), 4096, pose)
        .read(
            secrets.SystemRandom().randint(-99999, 99999),
            secrets.SystemRandom().randint(-99999, 99999),
            secrets.SystemRandom().uniform(-9.0, 9.0),
        )
        .position(),
        equal_to(pose[:2]),
        "first reading did not leave the odometry at its pose",
    )


def test_unread_keeps_its_heading_after_the_first_reading() -> None:
    """The first gyro angle only sets the start, whatever it is."""
    heading = secrets.SystemRandom().uniform(-math.pi, math.pi)
    assert_that(
        Unread(Differential(0.033, 0.288), 4096, (0.0, 0.0, heading))
        .read(0, 0, secrets.SystemRandom().uniform(-9.0, 9.0))
        .heading(),
        equal_to(heading),
        "first reading did not keep the heading of the odometry",
    )


def test_unread_moves_by_the_counts_after_the_first_reading() -> None:
    """The second reading moves by the counts since the first one."""
    start = secrets.SystemRandom().randint(-99999, 99999)
    ticks = secrets.SystemRandom().randint(1, 4096)
    assert_that(
        Unread(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0))
        .read(start, start, 0.0)
        .read(start + ticks, start + ticks, 0.0)
        .position()[0],
        close_to(ticks * 2 * math.pi / 4096 * 0.033, 1e-9),
        "second reading did not move by the counts since the first",
    )
