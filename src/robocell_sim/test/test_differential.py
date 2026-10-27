"""Tests for the differential drive kinematics."""

import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to
from robocell_sim.robot.implementations.drive.differential import Differential

from test.waffle import Waffle


def test_differential_slows_left_wheel_by_the_turn() -> None:
    """The left wheel rim runs at the forward speed minus the turn."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    track = secrets.SystemRandom().uniform(0.1, 0.5)
    linear = secrets.SystemRandom().uniform(-0.3, 0.3)
    angular = secrets.SystemRandom().uniform(-2.0, 2.0)
    assert_that(
        Differential(radius, track).wheels(linear, angular)[0],
        close_to((linear - angular * track / 2) / radius, 1e-9),
        "left wheel did not lag behind the turn",
    )


def test_differential_speeds_right_wheel_up_by_the_turn() -> None:
    """The right wheel rim runs at the forward speed plus the turn."""
    radius = secrets.SystemRandom().uniform(0.01, 0.1)
    track = secrets.SystemRandom().uniform(0.1, 0.5)
    linear = secrets.SystemRandom().uniform(-0.3, 0.3)
    angular = secrets.SystemRandom().uniform(-2.0, 2.0)
    assert_that(
        Differential(radius, track).wheels(linear, angular)[1],
        close_to((linear + angular * track / 2) / radius, 1e-9),
        "right wheel did not lead the turn",
    )


def test_differential_drives_waffle_pi_at_the_commanded_speed() -> None:
    """The wheel speeds move the simulated robot as fast as commanded."""
    linear = secrets.SystemRandom().uniform(0.05, 0.25)
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
            *Differential(0.033, 0.288).wheels(linear, 0.0),
            secrets.SystemRandom().uniform(-math.pi, math.pi),
        ).velocity()[0],
        close_to(linear, linear * 0.1),
        "robot did not drive at the commanded speed",
    )


def test_differential_turns_waffle_pi_at_the_commanded_yaw_rate() -> None:
    """The wheel speeds turn the simulated robot as fast as commanded."""
    angular = secrets.SystemRandom().uniform(0.3, 1.0)
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
            *Differential(0.033, 0.288).wheels(
                secrets.SystemRandom().uniform(0.05, 0.1), angular
            ),
            secrets.SystemRandom().uniform(-math.pi, math.pi),
        ).velocity()[1],
        close_to(angular, angular * 0.1),
        "robot did not turn at the commanded yaw rate",
    )


def test_differential_recovers_forward_speed_from_its_wheels() -> None:
    """The body velocity undoes the wheel speeds in forward speed."""
    drive = Differential(
        secrets.SystemRandom().uniform(0.01, 0.1),
        secrets.SystemRandom().uniform(0.1, 0.5),
    )
    linear = secrets.SystemRandom().uniform(-0.3, 0.3)
    assert_that(
        drive.velocity(
            *drive.wheels(linear, secrets.SystemRandom().uniform(-2.0, 2.0))
        )[0],
        close_to(linear, 1e-9),
        "forward speed did not survive the round trip through the wheels",
    )


def test_differential_recovers_yaw_rate_from_its_wheels() -> None:
    """The body velocity undoes the wheel speeds in yaw rate."""
    drive = Differential(
        secrets.SystemRandom().uniform(0.01, 0.1),
        secrets.SystemRandom().uniform(0.1, 0.5),
    )
    angular = secrets.SystemRandom().uniform(-2.0, 2.0)
    assert_that(
        drive.velocity(
            *drive.wheels(secrets.SystemRandom().uniform(-0.3, 0.3), angular)
        )[1],
        close_to(angular, 1e-9),
        "yaw rate did not survive the round trip through the wheels",
    )


def test_differential_reads_waffle_pi_yaw_rate_from_wheel_joints() -> None:
    """The wheel joint speeds tell how fast the simulated robot turns."""
    ride = Waffle(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        secrets.SystemRandom().uniform(1.0, 3.0),
        secrets.SystemRandom().uniform(4.0, 6.0),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    )
    assert_that(
        Differential(0.033, 0.288).velocity(*ride.wheels())[1],
        close_to(ride.velocity()[1], 0.05),
        "wheel joints did not tell the yaw rate of the robot",
    )
