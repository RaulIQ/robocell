"""Tests for the arrow key command."""

import secrets

import glfw
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.implementations.arrows import Arrows
from robocell_sim.implementations.stepped import Stepped


def test_arrows_speed_forward_on_up() -> None:
    """The up arrow raises the forward speed by one step."""
    step = secrets.SystemRandom().uniform(0.01, 0.1)
    assert_that(
        Arrows(Stepped(step, 5, 0), Stepped(0.3, 6, 0))
        .press(glfw.KEY_UP)
        .velocity()[0],
        close_to(step, 1e-9),
        "up arrow did not speed the robot forward",
    )


def test_arrows_slow_down_on_down() -> None:
    """The down arrow lowers the forward speed by one step."""
    step = secrets.SystemRandom().uniform(0.01, 0.1)
    assert_that(
        Arrows(Stepped(step, 5, 0), Stepped(0.3, 6, 0))
        .press(glfw.KEY_DOWN)
        .velocity()[0],
        close_to(-step, 1e-9),
        "down arrow did not slow the robot down",
    )


def test_arrows_turn_left_on_left() -> None:
    """The left arrow raises the counterclockwise yaw rate by one step."""
    step = secrets.SystemRandom().uniform(0.1, 0.5)
    assert_that(
        Arrows(Stepped(0.05, 5, 0), Stepped(step, 6, 0))
        .press(glfw.KEY_LEFT)
        .velocity()[1],
        close_to(step, 1e-9),
        "left arrow did not turn the robot left",
    )


def test_arrows_turn_right_on_right() -> None:
    """The right arrow lowers the counterclockwise yaw rate by one step."""
    step = secrets.SystemRandom().uniform(0.1, 0.5)
    assert_that(
        Arrows(Stepped(0.05, 5, 0), Stepped(step, 6, 0))
        .press(glfw.KEY_RIGHT)
        .velocity()[1],
        close_to(-step, 1e-9),
        "right arrow did not turn the robot right",
    )


def test_arrows_stop_on_enter() -> None:
    """The enter key brings both speeds to rest."""
    assert_that(
        Arrows(
            Stepped(0.05, 5, secrets.SystemRandom().randint(1, 5)),
            Stepped(0.3, 6, secrets.SystemRandom().randint(1, 6)),
        )
        .press(glfw.KEY_ENTER)
        .velocity(),
        equal_to((0.0, 0.0)),
        "enter did not stop the robot",
    )


def test_arrows_ignore_letter_keys() -> None:
    """A letter key leaves the commanded velocity as it was."""
    arrows = Arrows(
        Stepped(0.05, 5, secrets.SystemRandom().randint(-5, 5)),
        Stepped(0.3, 6, secrets.SystemRandom().randint(-6, 6)),
    )
    assert_that(
        arrows.press(
            secrets.SystemRandom().randint(ord("A"), ord("Z"))
        ).velocity(),
        equal_to(arrows.velocity()),
        "letter key did not leave the commanded velocity alone",
    )
