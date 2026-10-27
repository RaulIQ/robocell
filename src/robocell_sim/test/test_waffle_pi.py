"""Tests for the TurtleBot3 Waffle Pi model."""

import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to

from test.waffle import Waffle


def test_robot_drives_forward_at_wheel_speed_times_radius() -> None:
    """Both wheels spinning forward move the base at their rim speed."""
    speed = secrets.SystemRandom().uniform(2.0, 6.0)
    assert_that(
        Waffle(
            mujoco.MjModel.from_xml_path(
                str(
                    Path(__file__).parents[1]
                    / "models"
                    / "waffle_pi"
                    / "scene.xml"
                )
            ),
            speed,
            speed,
            secrets.SystemRandom().uniform(-math.pi, math.pi),
        ).velocity()[0],
        close_to(speed * 0.033, speed * 0.033 * 0.1),
        "base did not move at the wheel rim speed",
    )


def test_robot_spins_in_place_when_wheels_turn_apart() -> None:
    """Opposite wheel speeds turn the base around its vertical axis."""
    speed = secrets.SystemRandom().uniform(2.0, 6.0)
    assert_that(
        Waffle(
            mujoco.MjModel.from_xml_path(
                str(
                    Path(__file__).parents[1]
                    / "models"
                    / "waffle_pi"
                    / "scene.xml"
                )
            ),
            -speed,
            speed,
            secrets.SystemRandom().uniform(-math.pi, math.pi),
        ).velocity()[1],
        close_to(2 * speed * 0.033 / 0.288, 2 * speed * 0.033 / 0.288 * 0.1),
        "base did not turn at the rate the wheels demand",
    )


def test_accelerometer_reads_gravity_at_rest() -> None:
    """The accelerometer of a robot at rest reads gravity up its z axis."""
    assert_that(
        math.dist(
            Waffle(
                mujoco.MjModel.from_xml_path(
                    str(
                        Path(__file__).parents[1]
                        / "models"
                        / "waffle_pi"
                        / "scene.xml"
                    )
                ),
                0.0,
                0.0,
                secrets.SystemRandom().uniform(-math.pi, math.pi),
            )
            .state()
            .sensor("imu_accelerometer")
            .data,
            (0.0, 0.0, 9.81),
        ),
        close_to(0.0, 0.05),
        "accelerometer did not read gravity at rest",
    )
