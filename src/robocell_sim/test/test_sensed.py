"""Tests for the IMU that reads the MuJoCo sensors."""

import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to
from robocell_sim.hardware.implementations.imu.sensed import Sensed

from test.waffle import Waffle


def test_sensed_measures_the_yaw_rate_of_a_spinning_robot() -> None:
    """The rate about z is the turn rate the opposite wheels demand."""
    speed = secrets.SystemRandom().uniform(2.0, 6.0)
    assert_that(
        Sensed(
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
            ).state(),
            "imu_gyro",
            "imu_accelerometer",
        ).rate()[2],
        close_to(2 * speed * 0.033 / 0.288, 2 * speed * 0.033 / 0.288 * 0.1),
        "IMU did not measure the yaw rate of the spinning robot",
    )


def test_sensed_measures_gravity_at_rest() -> None:
    """The acceleration of a robot at rest is gravity up its z axis."""
    assert_that(
        math.dist(
            Sensed(
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
                ).state(),
                "imu_gyro",
                "imu_accelerometer",
            ).acceleration(),
            (0.0, 0.0, 9.81),
        ),
        close_to(0.0, 0.05),
        "IMU did not measure gravity at rest",
    )
