"""Tests for the step of MuJoCo physics alone."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to
from robocell_sim.hardware.implementations.physics.physical import Physical


def test_physical_moves_the_clock_by_one_step_each_time() -> None:
    """Every step moves the simulation clock by the model timestep."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    count = secrets.SystemRandom().randint(2, 300)
    for _ in range(count):
        Physical(model, data).take()
    assert_that(
        data.time,
        close_to(count * model.opt.timestep, 1e-9),
        "physics did not move the clock by one timestep per step",
    )
