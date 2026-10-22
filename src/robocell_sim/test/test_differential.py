"""Tests for the differential drive kinematics."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to
from robocell_sim.differential import Differential


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
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    linear = secrets.SystemRandom().uniform(0.05, 0.25)
    data.ctrl[
        [model.actuator("wheel_left").id, model.actuator("wheel_right").id]
    ] = Differential(0.033, 0.288).wheels(linear, 0.0)
    mujoco.mj_step(model, data, nstep=1500)
    assert_that(
        data.body("base_link").cvel[3],
        close_to(linear, linear * 0.1),
        "robot did not drive at the commanded speed",
    )


def test_differential_turns_waffle_pi_at_the_commanded_yaw_rate() -> None:
    """The wheel speeds turn the simulated robot as fast as commanded."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    angular = secrets.SystemRandom().uniform(0.3, 1.0)
    data.ctrl[
        [model.actuator("wheel_left").id, model.actuator("wheel_right").id]
    ] = Differential(0.033, 0.288).wheels(
        secrets.SystemRandom().uniform(0.05, 0.1), angular
    )
    mujoco.mj_step(model, data, nstep=1500)
    assert_that(
        data.body("base_link").cvel[2],
        close_to(angular, angular * 0.1),
        "robot did not turn at the commanded yaw rate",
    )
