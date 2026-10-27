"""Tests for the step that publishes a report after it."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to
from robocell_sim.hardware.implementations.physics.physical import Physical
from robocell_sim.hardware.implementations.physics.reported import Reported

from test.noted import Noted


def test_reported_steps_the_world_every_time() -> None:
    """The world moves one step on every take."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    count = secrets.SystemRandom().randint(50, 900)
    for _ in range(count):
        Reported(Physical(model, data), Noted([], data)).take()
    assert_that(
        data.time,
        close_to(count * model.opt.timestep, 1e-9),
        "world did not move on every step",
    )


def test_reported_publishes_after_the_step() -> None:
    """The report sees the world as the step left it."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=secrets.SystemRandom().randint(0, 99))
    start = data.time
    times: list[float] = []
    Reported(Physical(model, data), Noted(times, data)).take()
    assert_that(
        times[-1],
        close_to(start + model.opt.timestep, 1e-9),
        "report did not see the world after the step",
    )
