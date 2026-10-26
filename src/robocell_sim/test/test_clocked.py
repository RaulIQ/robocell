"""Tests for the report that publishes the simulated time."""

import secrets
from pathlib import Path
from typing import Any

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.ros.implementations.clocked import Clocked

from test.kept import Kept


def test_clocked_publishes_the_time_of_the_simulation() -> None:
    """The clock message carries the time the simulation reached."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    count = secrets.SystemRandom().randint(2, 900)
    mujoco.mj_step(model, data, nstep=count)
    Clocked(data, Kept(messages)).publish()
    assert_that(
        messages[-1].clock.sec + messages[-1].clock.nanosec * 1e-9,
        close_to(count * model.opt.timestep, 1e-9),
        "clock message did not carry the time of the simulation",
    )


def test_clocked_publishes_once_per_call() -> None:
    """Every publication sends exactly one clock message."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    count = secrets.SystemRandom().randint(2, 300)
    for _ in range(count):
        Clocked(data, Kept(messages)).publish()
    assert_that(
        len(messages),
        equal_to(count),
        "clock did not send one message per publication",
    )
