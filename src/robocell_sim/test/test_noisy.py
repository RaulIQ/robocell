"""Tests for the IMU that reads its sensors through their inaccuracies."""

import math
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import assert_that, close_to
from robocell_sim.hardware.implementations.imu.gaussian import Gaussian
from robocell_sim.hardware.implementations.imu.noisy import Noisy
from robocell_sim.hardware.implementations.imu.sensed import Sensed

from test.waffle import Waffle


def test_noisy_reads_the_rate_through_the_gyro_inaccuracy() -> None:
    """The rate comes out shifted by the bias of the gyro alone."""
    speed = secrets.SystemRandom().uniform(-6.0, 6.0)
    data = Waffle(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        -speed,
        speed,
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    bias = (
        secrets.SystemRandom().uniform(-0.02, 0.02),
        secrets.SystemRandom().uniform(-0.02, 0.02),
        secrets.SystemRandom().uniform(-0.02, 0.02),
    )
    assert_that(
        math.dist(
            Noisy(
                Sensed(data, "imu_gyro", "imu_accelerometer"),
                Gaussian(
                    bias, 0.0, np.random.default_rng(secrets.randbits(64))
                ),
                Gaussian(
                    (0.3, 0.3, 0.3),
                    0.0,
                    np.random.default_rng(secrets.randbits(64)),
                ),
            ).rate(),
            np.add(Sensed(data, "imu_gyro", "imu_accelerometer").rate(), bias),
        ),
        close_to(0.0, 1e-12),
        "IMU did not read the rate through the gyro inaccuracy",
    )


def test_noisy_reads_the_acceleration_through_its_own_inaccuracy() -> None:
    """The acceleration comes out shifted by the accelerometer bias alone."""
    speed = secrets.SystemRandom().uniform(-6.0, 6.0)
    data = Waffle(
        mujoco.MjModel.from_xml_path(
            str(
                Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml"
            )
        ),
        speed,
        speed,
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    bias = (
        secrets.SystemRandom().uniform(-0.05, 0.05),
        secrets.SystemRandom().uniform(-0.05, 0.05),
        secrets.SystemRandom().uniform(-0.05, 0.05),
    )
    assert_that(
        math.dist(
            Noisy(
                Sensed(data, "imu_gyro", "imu_accelerometer"),
                Gaussian(
                    (0.3, 0.3, 0.3),
                    0.0,
                    np.random.default_rng(secrets.randbits(64)),
                ),
                Gaussian(
                    bias, 0.0, np.random.default_rng(secrets.randbits(64))
                ),
            ).acceleration(),
            np.add(
                Sensed(data, "imu_gyro", "imu_accelerometer").acceleration(),
                bias,
            ),
        ),
        close_to(0.0, 1e-12),
        "IMU did not read the acceleration through its own inaccuracy",
    )
