"""Tests for the gyro angle kept in the simulation user data."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.accumulated import Accumulated


def test_accumulated_reads_the_angle_from_its_slot() -> None:
    """The gyro reports the angle that the user data slot holds."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    angle = secrets.SystemRandom().uniform(-50.0, 50.0)
    data.userdata[0] = angle
    assert_that(
        Accumulated(data, 0).angle(),
        equal_to(angle),
        "gyro did not report the angle its slot holds",
    )
