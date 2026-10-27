"""Tests for the step that publishes a report once per period."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.physics.periodic import Periodic
from robocell_sim.hardware.implementations.physics.physical import Physical

from test.noted import Noted


def test_periodic_publishes_once_per_period() -> None:
    """The report goes out once on every whole period of simulated time."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    every = secrets.SystemRandom().randint(2, 40)
    count = secrets.SystemRandom().randint(50, 900)
    times: list[float] = []
    for _ in range(count):
        Periodic(
            Physical(model, data),
            data,
            every * model.opt.timestep,
            Noted(times, data),
        ).take()
    assert_that(
        len(times),
        equal_to(count // every),
        "report did not go out once per period",
    )


def test_periodic_publishes_on_the_boundaries_of_the_period() -> None:
    """Every report falls on a whole multiple of the period."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    every = secrets.SystemRandom().randint(2, 40)
    times: list[float] = []
    for _ in range(secrets.SystemRandom().randint(50, 900)):
        Periodic(
            Physical(model, data),
            data,
            every * model.opt.timestep,
            Noted(times, data),
        ).take()
    assert_that(
        {round(time / model.opt.timestep) % every for time in times},
        equal_to({0}),
        "report did not fall on the boundaries of the period",
    )


def test_periodic_steps_the_world_between_reports() -> None:
    """The world moves on every step, whether the report goes out or not."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    count = secrets.SystemRandom().randint(50, 900)
    for _ in range(count):
        Periodic(
            Physical(model, data),
            data,
            secrets.SystemRandom().randint(2, 40) * model.opt.timestep,
            Noted([], data),
        ).take()
    assert_that(
        data.time,
        close_to(count * model.opt.timestep, 1e-9),
        "world did not move on every step",
    )
