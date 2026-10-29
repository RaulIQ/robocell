"""Tests for individual clockwise lidar rays in a changing scene."""

import math
from pathlib import Path

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.lidar.directed import Directed


def test_directed_ranges_clockwise_from_the_robot() -> None:
    """270 clockwise degrees point to the left wall."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models/waffle_pi/walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Directed(data, "base_scan", (0.15, 12.0)).distance(270.0),
        close_to(1.45, 1e-6),
        "clockwise ray did not meet the left wall",
    )


def test_directed_reads_the_world_again_for_each_ray() -> None:
    """Moving between rays changes the distance within the same rotation."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models/waffle_pi/walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    beam = Directed(data, "base_scan", (0.15, 12.0))
    before = beam.distance(0.0)
    data.joint("base").qpos[0] += 0.237
    mujoco.mj_forward(model, data)
    assert_that(
        before - beam.distance(0.0),
        close_to(0.237, 1e-6),
        "later ray did not see the changed robot position",
    )


def test_directed_rejects_returns_outside_its_range() -> None:
    """The far wall cannot become a clipped near obstacle."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models/waffle_pi/walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Directed(data, "base_scan", (0.15, 0.713)).distance(0.0),
        equal_to(math.inf),
        "out of range return did not become invalid",
    )
