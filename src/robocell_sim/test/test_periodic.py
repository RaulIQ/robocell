"""Tests for the report that goes out once per period."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.physics.periodic import Periodic

from test.noted import Noted


def test_every_publishes_once_per_period() -> None:
    """The report goes out once on every whole period of simulated time."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    every = secrets.SystemRandom().randint(2, 40)
    count = secrets.SystemRandom().randint(50, 900)
    times: list[float] = []
    for _ in range(count):
        mujoco.mj_step(model, data)
        Periodic(data, every * model.opt.timestep, Noted(times, data)).publish()
    assert_that(
        len(times),
        equal_to(count // every),
        "report did not go out once per period",
    )


def test_every_publishes_on_the_boundaries_of_the_period() -> None:
    """Every report falls on a whole multiple of the period."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    every = secrets.SystemRandom().randint(2, 40)
    times: list[float] = []
    for _ in range(secrets.SystemRandom().randint(50, 900)):
        mujoco.mj_step(model, data)
        Periodic(data, every * model.opt.timestep, Noted(times, data)).publish()
    assert_that(
        {round(time / model.opt.timestep) % every for time in times},
        equal_to({0}),
        "report did not fall on the boundaries of the period",
    )
