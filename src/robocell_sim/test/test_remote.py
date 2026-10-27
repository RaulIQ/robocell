"""Tests for the command that a ROS 2 operator steers."""

import secrets

import glfw
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.command.arrows import Arrows
from robocell_sim.robot.implementations.command.remote import Remote
from robocell_sim.robot.implementations.drive.stepped import Stepped


def test_remote_drives_at_the_steered_velocity() -> None:
    """The command reports the forward speed and yaw rate it was steered to."""
    velocity = (
        secrets.SystemRandom().uniform(-0.26, 0.26),
        secrets.SystemRandom().uniform(-1.8, 1.8),
    )
    assert_that(
        Remote(
            Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0)), *velocity
        ).velocity(),
        equal_to(velocity),
        "remote command did not drive at the steered velocity",
    )


def test_remote_replaces_the_velocity_on_another_steer() -> None:
    """A newer steer overrides the velocity of the earlier one."""
    velocity = (
        secrets.SystemRandom().uniform(-0.26, 0.26),
        secrets.SystemRandom().uniform(-1.8, 1.8),
    )
    assert_that(
        Remote(
            Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0)),
            secrets.SystemRandom().uniform(-0.26, 0.26),
            secrets.SystemRandom().uniform(-1.8, 1.8),
        )
        .steer(*velocity)
        .velocity(),
        equal_to(velocity),
        "remote command did not take the newer velocity",
    )


def test_remote_hands_control_back_to_the_arrows_on_a_key() -> None:
    """An arrow key resumes the arrows from where the operator left them."""
    level = secrets.SystemRandom().randint(-4, 4)
    assert_that(
        Remote(
            Arrows(Stepped(0.05, 5, level), Stepped(0.3, 6, 0)),
            secrets.SystemRandom().uniform(-0.26, 0.26),
            secrets.SystemRandom().uniform(-1.8, 1.8),
        )
        .press(glfw.KEY_UP)
        .velocity()[0],
        close_to(0.05 * (level + 1), 1e-9),
        "an arrow key did not hand control back to the arrows",
    )


def test_remote_forgets_earlier_steers_on_a_key() -> None:
    """After several steers, one key press still returns to the arrows."""
    level = secrets.SystemRandom().randint(-4, 4)
    assert_that(
        Remote(
            Arrows(Stepped(0.05, 5, level), Stepped(0.3, 6, 0)),
            secrets.SystemRandom().uniform(-0.26, 0.26),
            secrets.SystemRandom().uniform(-1.8, 1.8),
        )
        .steer(
            secrets.SystemRandom().uniform(-0.26, 0.26),
            secrets.SystemRandom().uniform(-1.8, 1.8),
        )
        .press(glfw.KEY_UP)
        .velocity()[0],
        close_to(0.05 * (level + 1), 1e-9),
        "a key press after several steers did not return to the arrows",
    )
