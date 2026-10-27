"""Tests for the step that integrates the gyro into a heading."""

import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to
from robocell_sim.hardware.implementations.imu.integrated import Integrated
from robocell_sim.hardware.implementations.physics.physical import Physical


def test_integrated_follows_the_heading_of_a_turning_robot() -> None:
    """The integrated gyro angle matches the yaw of a robot driving an arc."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    data.ctrl = (
        secrets.SystemRandom().uniform(-6.0, -2.0),
        secrets.SystemRandom().uniform(2.0, 6.0),
    )
    for _ in range(1500):
        Integrated(Physical(model, data), data, "imu_gyro", 0).take()
    quat = data.body("base_link").xquat
    assert_that(
        math.remainder(
            data.userdata[0]
            - math.atan2(
                2 * (quat[0] * quat[3] + quat[1] * quat[2]),
                1 - 2 * (quat[2] ** 2 + quat[3] ** 2),
            ),
            2 * math.pi,
        ),
        close_to(0.0, 0.01),
        "integrated gyro angle did not follow the heading of the robot",
    )


def test_integrated_still_takes_the_inner_step() -> None:
    """Integrating the gyro moves the simulation by the step it wraps."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    count = secrets.SystemRandom().randint(2, 300)
    for _ in range(count):
        Integrated(Physical(model, data), data, "imu_gyro", 0).take()
    assert_that(
        data.time,
        close_to(count * model.opt.timestep, 1e-9),
        "integrating the gyro did not take the inner step",
    )
