"""Tests for the physics that step up to a moment in time."""

import secrets
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.implementations.paced import Paced
from robocell_sim.implementations.physical import Physical


def test_paced_reaches_the_moment_within_one_step() -> None:
    """Advancing steps the physics until its clock reaches the moment."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    moment = secrets.SystemRandom().uniform(0.01, 2.0)
    Paced(Physical(model, data), data).advance(moment)
    assert_that(
        data.time - moment,
        close_to(model.opt.timestep / 2, model.opt.timestep / 2),
        "physics did not stop within one step past the moment",
    )


def test_paced_keeps_pace_over_many_frames() -> None:
    """Many short frames end as close to the last moment as one long one."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    frame = secrets.SystemRandom().uniform(0.0021, 0.0059)
    for count in range(1, 301):
        Paced(Physical(model, data), data).advance(frame * count)
    assert_that(
        data.time - frame * 300,
        close_to(model.opt.timestep / 2, model.opt.timestep / 2),
        "physics did not keep pace with the clock over many frames",
    )


def test_paced_leaves_a_passed_moment_alone() -> None:
    """A moment the physics has already passed takes no step."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_step(model, data, nstep=secrets.SystemRandom().randint(10, 99))
    time = data.time
    Paced(Physical(model, data), data).advance(
        secrets.SystemRandom().uniform(0.0, time)
    )
    assert_that(
        data.time,
        equal_to(time),
        "physics did not stay put for a moment it had passed",
    )
