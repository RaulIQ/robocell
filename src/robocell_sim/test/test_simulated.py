"""Tests for the wheel read from a MuJoCo encoder sensor."""

import math
import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to, equal_to, less_than
from robocell_sim.hardware.implementations.simulated import Simulated


def test_simulated_counts_whole_ticks_of_the_wheel_angle() -> None:
    """The encoder turns the wheel angle into whole ticks of its resolution."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    angle = secrets.SystemRandom().uniform(0.1, 50.0)
    resolution = secrets.SystemRandom().randint(1024, 8192)
    data.joint("wheel_left_joint").qpos = angle
    mujoco.mj_forward(model, data)
    assert_that(
        Simulated(data, "wheel_left_encoder", resolution).ticks(),
        equal_to(math.floor(angle * resolution / (2 * math.pi))),
        "encoder did not count the whole ticks of the wheel angle",
    )


def test_simulated_counts_backward_turns_below_zero() -> None:
    """A wheel turned backwards reads a negative count."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    data.joint("wheel_left_joint").qpos = secrets.SystemRandom().uniform(
        -50.0, -0.1
    )
    mujoco.mj_forward(model, data)
    assert_that(
        Simulated(data, "wheel_left_encoder", 4096).ticks(),
        less_than(0),
        "encoder did not count a backward turn below zero",
    )


def test_simulated_follows_the_right_wheel_while_driving() -> None:
    """The right encoder tracks the right wheel joint of a driving robot."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    data.ctrl = (
        secrets.SystemRandom().uniform(1.0, 3.0),
        secrets.SystemRandom().uniform(4.0, 6.0),
    )
    mujoco.mj_step(model, data, nstep=1500)
    mujoco.mj_forward(model, data)
    assert_that(
        Simulated(data, "wheel_right_encoder", 4096).ticks(),
        equal_to(
            math.floor(
                data.joint("wheel_right_joint").qpos[0] * 4096 / (2 * math.pi)
            )
        ),
        "right encoder did not follow the right wheel",
    )


def test_simulated_turns_whole_ticks_into_radians() -> None:
    """The angle of the wheel is its whole ticks over the resolution."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    angle = secrets.SystemRandom().uniform(-50.0, 50.0)
    resolution = secrets.SystemRandom().randint(1024, 8192)
    data.joint("wheel_right_joint").qpos = angle
    mujoco.mj_forward(model, data)
    assert_that(
        Simulated(data, "wheel_right_encoder", resolution).angle(),
        close_to(
            math.floor(angle * resolution / (2 * math.pi))
            * 2
            * math.pi
            / resolution,
            1e-9,
        ),
        "encoder did not turn whole ticks into radians",
    )
