"""Tests for the TurtleBot3 Waffle Pi model."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to


def test_robot_drives_forward_at_wheel_speed_times_radius() -> None:
    """Both wheels spinning forward move the base at their rim speed."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    speed = secrets.SystemRandom().uniform(2.0, 6.0)
    data.actuator("wheel_left").ctrl = speed
    data.actuator("wheel_right").ctrl = speed
    mujoco.mj_step(model, data, nstep=1500)
    assert_that(
        data.body("base_link").cvel[3],
        close_to(speed * 0.033, speed * 0.033 * 0.1),
        "base did not move at the wheel rim speed",
    )


def test_robot_spins_in_place_when_wheels_turn_apart() -> None:
    """Opposite wheel speeds turn the base around its vertical axis."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    speed = secrets.SystemRandom().uniform(2.0, 6.0)
    data.actuator("wheel_left").ctrl = -speed
    data.actuator("wheel_right").ctrl = speed
    mujoco.mj_step(model, data, nstep=1500)
    assert_that(
        data.body("base_link").cvel[2],
        close_to(2 * speed * 0.033 / 0.288, 2 * speed * 0.033 / 0.288 * 0.1),
        "base did not turn at the rate the wheels demand",
    )
