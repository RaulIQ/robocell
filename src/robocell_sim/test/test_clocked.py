"""Tests for the step that publishes the simulated time."""

import secrets
from pathlib import Path
from typing import Any

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.physical import Physical
from robocell_sim.ros.implementations.clocked import Clocked

from test.kept import Kept


def test_clocked_publishes_the_time_of_the_simulation() -> None:
    """The last clock message carries the time the steps reached."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    count = secrets.SystemRandom().randint(2, 900)
    for _ in range(count):
        Clocked(Physical(model, data), data, Kept(messages)).take()
    assert_that(
        messages[-1].clock.sec + messages[-1].clock.nanosec * 1e-9,
        close_to(count * model.opt.timestep, 1e-9),
        "clock message did not carry the time of the simulation",
    )


def test_clocked_publishes_once_per_step() -> None:
    """Every step of the simulation sends exactly one clock message."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    count = secrets.SystemRandom().randint(2, 300)
    for _ in range(count):
        Clocked(Physical(model, data), data, Kept(messages)).take()
    assert_that(
        len(messages),
        equal_to(count),
        "clock did not publish one message per step",
    )
