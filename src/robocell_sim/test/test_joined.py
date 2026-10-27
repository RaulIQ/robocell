"""Tests for the report that publishes several reports."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.physics.joined import Joined

from test.noted import Noted


def test_joined_publishes_every_report_in_order() -> None:
    """Each report goes out once, in the order it was given."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    states = [
        mujoco.MjData(model)
        for _ in range(secrets.SystemRandom().randint(1, 9))
    ]
    for state in states:
        mujoco.mj_step(
            model, state, nstep=secrets.SystemRandom().randint(1, 99)
        )
    times: list[float] = []
    Joined(tuple(Noted(times, state) for state in states)).publish()
    assert_that(
        times,
        equal_to([state.time for state in states]),
        "joined report did not publish every report once in order",
    )
