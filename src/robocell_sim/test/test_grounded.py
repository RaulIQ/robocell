"""Tests for the report that publishes the footprint of the robot to TF."""

import math
import secrets
from pathlib import Path
from typing import Any

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.ros.implementations.pose.grounded import Grounded

from test.kept import Kept
from test.waffle import Waffle


def test_grounded_links_the_footprint_to_odom() -> None:
    """The transform runs from the odom frame to base_footprint."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    Grounded(data, "base_link", Kept(messages)).publish()
    assert_that(
        (
            messages[-1].transforms[0].header.frame_id,
            messages[-1].transforms[0].child_frame_id,
        ),
        equal_to(("odom", "base_footprint")),
        "transform did not link base_footprint to odom",
    )


def test_grounded_puts_the_footprint_on_the_floor_under_the_body() -> None:
    """The footprint lies at zero height right below the body."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = Waffle(
        model,
        secrets.SystemRandom().uniform(-6.0, 6.0),
        secrets.SystemRandom().uniform(-6.0, 6.0),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    messages: list[Any] = []
    Grounded(data, "base_link", Kept(messages)).publish()
    assert_that(
        math.dist(
            (
                messages[-1].transforms[0].transform.translation.x,
                messages[-1].transforms[0].transform.translation.y,
                messages[-1].transforms[0].transform.translation.z,
            ),
            (
                data.body("base_link").xpos[0],
                data.body("base_link").xpos[1],
                0.0,
            ),
        ),
        close_to(0.0, 1e-9),
        "transform did not put the footprint on the floor under the body",
    )


def test_grounded_turns_the_footprint_to_the_heading_of_the_body() -> None:
    """The footprint keeps only the yaw of the body around the vertical."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = Waffle(
        model,
        secrets.SystemRandom().uniform(-6.0, 6.0),
        secrets.SystemRandom().uniform(-6.0, 6.0),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    messages: list[Any] = []
    Grounded(data, "base_link", Kept(messages)).publish()
    heading = math.atan2(
        data.body("base_link").xmat[3], data.body("base_link").xmat[0]
    )
    assert_that(
        math.dist(
            (
                messages[-1].transforms[0].transform.rotation.w,
                messages[-1].transforms[0].transform.rotation.x,
                messages[-1].transforms[0].transform.rotation.y,
                messages[-1].transforms[0].transform.rotation.z,
            ),
            (math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2)),
        ),
        close_to(0.0, 1e-9),
        "transform did not turn the footprint to the heading of the body",
    )


def test_grounded_stamps_the_transform_with_the_simulated_time() -> None:
    """The transform carries the time of the simulation it publishes."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    mujoco.mj_step(model, data, nstep=secrets.SystemRandom().randint(2, 900))
    Grounded(data, "base_link", Kept(messages)).publish()
    assert_that(
        messages[-1].transforms[0].header.stamp.sec
        + messages[-1].transforms[0].header.stamp.nanosec * 1e-9,
        close_to(data.time, 1e-9),
        "transform did not carry the simulated time",
    )
