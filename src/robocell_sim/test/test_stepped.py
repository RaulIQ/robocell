"""Tests for the speed that changes in equal steps."""

import secrets

from hamcrest import assert_that, close_to
from robocell_sim.robot.implementations.drive.stepped import Stepped


def test_stepped_rises_by_one_step() -> None:
    """Speeding up adds one step to the speed below the limit."""
    step = secrets.SystemRandom().uniform(0.01, 0.5)
    top = secrets.SystemRandom().randint(3, 9)
    level = secrets.SystemRandom().randint(-top, top - 1)
    assert_that(
        Stepped(step, top, level).up().value(),
        close_to(step * (level + 1), 1e-9),
        "speed did not rise by one step",
    )


def test_stepped_holds_at_the_top_limit() -> None:
    """Speeding up at the limit keeps the speed at the limit."""
    step = secrets.SystemRandom().uniform(0.01, 0.5)
    top = secrets.SystemRandom().randint(3, 9)
    assert_that(
        Stepped(step, top, top).up().value(),
        close_to(step * top, 1e-9),
        "speed did not hold at its top limit",
    )


def test_stepped_falls_by_one_step() -> None:
    """Slowing down takes one step off the speed above the limit."""
    step = secrets.SystemRandom().uniform(0.01, 0.5)
    top = secrets.SystemRandom().randint(3, 9)
    level = secrets.SystemRandom().randint(1 - top, top)
    assert_that(
        Stepped(step, top, level).down().value(),
        close_to(step * (level - 1), 1e-9),
        "speed did not fall by one step",
    )


def test_stepped_holds_at_the_bottom_limit() -> None:
    """Slowing down at the reverse limit keeps the speed there."""
    step = secrets.SystemRandom().uniform(0.01, 0.5)
    top = secrets.SystemRandom().randint(3, 9)
    assert_that(
        Stepped(step, top, -top).down().value(),
        close_to(-step * top, 1e-9),
        "speed did not hold at its reverse limit",
    )


def test_stepped_stops_at_zero() -> None:
    """Stopping brings any speed to rest."""
    top = secrets.SystemRandom().randint(3, 9)
    assert_that(
        Stepped(
            secrets.SystemRandom().uniform(0.01, 0.5),
            top,
            secrets.SystemRandom().randint(-top, top),
        )
        .stop()
        .value(),
        close_to(0.0, 1e-9),
        "speed did not come to rest",
    )
