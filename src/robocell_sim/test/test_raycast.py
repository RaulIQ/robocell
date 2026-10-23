"""Tests for the lidar that casts MuJoCo rays."""

import math
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.raycast import Raycast


def test_raycast_ranges_the_wall_ahead_at_index_zero() -> None:
    """The first ray points forward and meets the front wall face."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Raycast(model, data, "base_scan", (0.12, 3.5)).ranges()[0],
        close_to(0.95 + 0.064, 1e-6),
        "first ray did not range the wall ahead",
    )


def test_raycast_ranges_the_wall_on_the_left_at_index_ninety() -> None:
    """Angles grow counterclockwise, so index 90 looks to the left."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Raycast(model, data, "base_scan", (0.12, 3.5)).ranges()[90],
        close_to(1.45, 1e-6),
        "ray at index ninety did not range the wall on the left",
    )


def test_raycast_reads_infinity_where_nothing_echoes() -> None:
    """A ray that meets nothing behind the robot reads infinity."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Raycast(model, data, "base_scan", (0.12, 3.5)).ranges()[180],
        equal_to(math.inf),
        "ray into the empty space behind did not read infinity",
    )


def test_raycast_never_sees_the_robot_itself() -> None:
    """On the empty floor every ray reads infinity."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        set(Raycast(model, data, "base_scan", (0.12, 3.5)).ranges()),
        equal_to({math.inf}),
        "lidar did not stay blind to the robot itself",
    )


def test_raycast_turns_with_the_robot() -> None:
    """After a quarter turn left the front wall shows up at index 270."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mju_axisAngle2Quat(
        data.joint("base").qpos[3:], np.array([0.0, 0.0, 1.0]), math.pi / 2
    )
    mujoco.mj_forward(model, data)
    assert_that(
        Raycast(model, data, "base_scan", (0.12, 3.5)).ranges()[270],
        close_to(0.95, 1e-6),
        "front wall did not move to the right after a quarter turn",
    )


def test_raycast_ignores_an_obstacle_too_close() -> None:
    """An obstacle closer than the shortest range reads infinity."""
    spec = mujoco.MjSpec.from_file(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.005, 0.3, 0.3],
        pos=[-0.064 + 0.05 + 0.005, 0.0, 0.3],
    )
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Raycast(model, data, "base_scan", (0.12, 3.5)).ranges()[0],
        equal_to(math.inf),
        "an obstacle five centimeters away did not read infinity",
    )


def test_raycast_ignores_a_wall_beyond_the_longest_range() -> None:
    """A wall farther than the longest range reads infinity."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        Raycast(
            model,
            data,
            "base_scan",
            (0.12, secrets.SystemRandom().uniform(0.2, 1.0)),
        ).ranges()[0],
        equal_to(math.inf),
        "a wall beyond the longest range did not read infinity",
    )


def test_raycast_casts_one_ray_per_degree() -> None:
    """A scan holds one range for every degree around the robot."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "walls.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert_that(
        len(Raycast(model, data, "base_scan", (0.12, 3.5)).ranges()),
        equal_to(360),
        "scan did not hold one range per degree",
    )
