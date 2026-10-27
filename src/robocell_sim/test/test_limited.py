"""Tests for the kinematics that keep the wheels within their limit."""

import secrets

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.drive.limited import Limited


def test_limited_holds_the_fastest_wheel_at_the_limit() -> None:
    """A velocity beyond the limit brings the fastest wheel down to it."""
    assert_that(
        max(
            map(
                abs,
                Limited(Differential(0.033, 0.288), 7.88).wheels(
                    secrets.SystemRandom().uniform(0.2, 0.3),
                    secrets.SystemRandom().uniform(0.8, 1.8),
                ),
            )
        ),
        close_to(7.88, 1e-9),
        "fastest wheel did not stop at the limit",
    )


def test_limited_keeps_the_commanded_arc() -> None:
    """Scaling the wheels down keeps the yaw rate over the forward speed."""
    drive = Differential(0.033, 0.288)
    linear = secrets.SystemRandom().uniform(0.2, 0.3)
    angular = secrets.SystemRandom().uniform(0.8, 1.8)
    velocity = drive.velocity(*Limited(drive, 7.88).wheels(linear, angular))
    assert_that(
        velocity[1] / velocity[0],
        close_to(angular / linear, 1e-9),
        "limited wheels did not keep the commanded arc",
    )


def test_limited_leaves_slow_wheels_alone() -> None:
    """A velocity within the limit passes through unchanged."""
    drive = Differential(0.033, 0.288)
    linear = secrets.SystemRandom().uniform(-0.1, 0.1)
    angular = secrets.SystemRandom().uniform(-0.5, 0.5)
    assert_that(
        Limited(drive, 7.88).wheels(linear, angular),
        equal_to(drive.wheels(linear, angular)),
        "wheels within the limit did not pass through unchanged",
    )


def test_limited_reads_velocity_through_its_origin() -> None:
    """The body velocity of measured wheels comes from the wrapped drive."""
    drive = Differential(0.033, 0.288)
    left = secrets.SystemRandom().uniform(-10.0, 10.0)
    right = secrets.SystemRandom().uniform(-10.0, 10.0)
    assert_that(
        Limited(drive, 7.88).velocity(left, right),
        equal_to(drive.velocity(left, right)),
        "measured wheels did not reach the wrapped drive",
    )
