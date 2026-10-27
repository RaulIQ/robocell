"""Tests for the command that holds the robot back from lidar echoes."""

import math
import secrets
from pathlib import Path

import glfw
import mujoco
from hamcrest import assert_that, close_to, equal_to, greater_than
from robocell_sim.hardware.implementations.lidar.raycast import Raycast
from robocell_sim.robot.implementations.command.arrows import Arrows
from robocell_sim.robot.implementations.command.guarded import Guarded
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.drive.limited import Limited
from robocell_sim.robot.implementations.drive.stepped import Stepped


def test_guarded_lets_the_robot_drive_on_a_clear_path() -> None:
    """With no echo around, the forward speed passes unchanged."""
    level = secrets.SystemRandom().randint(1, 5)
    assert_that(
        Guarded(
            Arrows(Stepped(0.05, 5, level), Stepped(0.3, 6, 0)),
            tuple(math.inf for _ in range(360)),
            0.3,
            0.2,
        ).velocity()[0],
        close_to(0.05 * level, 1e-9),
        "a clear path did not let the forward speed pass",
    )


def test_guarded_stops_the_robot_before_an_echo_ahead() -> None:
    """An echo inside the corridor ahead drops the forward speed to zero."""
    scan = [math.inf] * 360
    scan[secrets.SystemRandom().randint(-20, 20)] = (
        secrets.SystemRandom().uniform(0.12, 0.28)
    )
    assert_that(
        Guarded(
            Arrows(
                Stepped(0.05, 5, secrets.SystemRandom().randint(1, 5)),
                Stepped(0.3, 6, 0),
            ),
            tuple(scan),
            0.3,
            0.2,
        ).velocity()[0],
        equal_to(0.0),
        "an echo ahead did not stop the robot",
    )


def test_guarded_keeps_the_turn_when_blocked_ahead() -> None:
    """An echo ahead leaves the yaw rate alone so the robot can steer away."""
    level = secrets.SystemRandom().choice((-6, -3, 3, 6))
    scan = [math.inf] * 360
    scan[0] = 0.2
    assert_that(
        Guarded(
            Arrows(Stepped(0.05, 5, 5), Stepped(0.3, 6, level)),
            tuple(scan),
            0.3,
            0.2,
        ).velocity()[1],
        close_to(0.3 * level, 1e-9),
        "an echo ahead did not leave the yaw rate alone",
    )


def test_guarded_lets_the_robot_back_away_from_an_echo_ahead() -> None:
    """An echo ahead still lets the robot reverse."""
    level = secrets.SystemRandom().randint(1, 5)
    scan = [math.inf] * 360
    scan[0] = 0.2
    assert_that(
        Guarded(
            Arrows(Stepped(0.05, 5, -level), Stepped(0.3, 6, 0)),
            tuple(scan),
            0.3,
            0.2,
        ).velocity()[0],
        close_to(-0.05 * level, 1e-9),
        "an echo ahead did not let the robot reverse",
    )


def test_guarded_stops_the_robot_before_an_echo_behind() -> None:
    """An echo inside the corridor behind drops the reverse speed to zero."""
    scan = [math.inf] * 360
    scan[180 + secrets.SystemRandom().randint(-20, 20)] = (
        secrets.SystemRandom().uniform(0.12, 0.28)
    )
    assert_that(
        Guarded(
            Arrows(
                Stepped(0.05, 5, -secrets.SystemRandom().randint(1, 5)),
                Stepped(0.3, 6, 0),
            ),
            tuple(scan),
            0.3,
            0.2,
        ).velocity()[0],
        equal_to(0.0),
        "an echo behind did not stop the reversing robot",
    )


def test_guarded_ignores_an_echo_beside_the_corridor() -> None:
    """An echo ahead but wider than the corridor leaves the speed alone."""
    scan = [math.inf] * 360
    scan[secrets.SystemRandom().randint(60, 120)] = (
        secrets.SystemRandom().uniform(0.25, 0.3)
    )
    assert_that(
        Guarded(
            Arrows(Stepped(0.05, 5, 5), Stepped(0.3, 6, 0)),
            tuple(scan),
            0.3,
            0.2,
        ).velocity()[0],
        close_to(0.25, 1e-9),
        "an echo beside the corridor did not leave the speed alone",
    )


def test_guarded_ignores_an_echo_beyond_its_reach() -> None:
    """An echo ahead past the reach of the corridor leaves the speed alone."""
    scan = [math.inf] * 360
    scan[0] = secrets.SystemRandom().uniform(0.31, 3.5)
    assert_that(
        Guarded(
            Arrows(Stepped(0.05, 5, 5), Stepped(0.3, 6, 0)),
            tuple(scan),
            0.3,
            0.2,
        ).velocity()[0],
        close_to(0.25, 1e-9),
        "an echo beyond the reach did not leave the speed alone",
    )


def test_guarded_passes_key_presses_to_the_operator() -> None:
    """Pressing a key changes the command it guards."""
    assert_that(
        Guarded(
            Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0)),
            tuple(math.inf for _ in range(360)),
            0.3,
            0.2,
        )
        .press(glfw.KEY_UP)
        .velocity()[0],
        close_to(0.05, 1e-9),
        "a key press did not reach the guarded command",
    )


def test_guarded_keeps_the_robot_off_the_wall_ahead() -> None:
    """A robot driving at full speed at a wall halts five cm short of it."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    for sample in range(30):
        data.ctrl = Limited(Differential(0.033, 0.288), 7.88).wheels(
            *Guarded(
                Arrows(Stepped(0.05, 5, 5), Stepped(0.3, 6, 0)),
                Raycast(model, data, "base_scan", (0.12, 3.5)).ranges(),
                0.3,
                0.2,
            ).velocity()
        )
        while data.time < (sample + 1) * 0.2:
            mujoco.mj_step(model, data)
    assert_that(
        0.95 - (data.body("base_link").xpos[0] + 0.0855),
        greater_than(0.05),
        "robot did not halt short of the wall ahead",
    )


def test_guarded_stops_a_steered_robot_before_an_echo_ahead() -> None:
    """A steer from ROS 2 cannot drive the robot into an echo ahead."""
    scan = [math.inf] * 360
    scan[secrets.SystemRandom().randint(-20, 20)] = (
        secrets.SystemRandom().uniform(0.12, 0.28)
    )
    assert_that(
        Guarded(
            Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0)),
            tuple(scan),
            0.3,
            0.2,
        )
        .steer(
            secrets.SystemRandom().uniform(0.01, 0.26),
            secrets.SystemRandom().uniform(-1.8, 1.8),
        )
        .velocity()[0],
        equal_to(0.0),
        "a steer did not stop before an echo ahead",
    )
