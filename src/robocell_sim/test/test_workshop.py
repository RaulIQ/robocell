"""Tests for the workshop room the robot drives in."""

import math
import secrets
from pathlib import Path

import mujoco
import numpy as np
from hamcrest import (
    all_of,
    assert_that,
    close_to,
    equal_to,
    greater_than,
    less_than,
)


def test_workshop_walls_close_the_room() -> None:
    """A level ray from the middle of the room meets the furniture or a wall."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "workshop.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    angle = secrets.SystemRandom().uniform(-math.pi, math.pi)
    assert_that(
        mujoco.mj_ray(
            model,
            data,
            np.array([0.0, 0.0, 0.3]),
            np.array([math.cos(angle), math.sin(angle), 0.0]),
            np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8),
            flg_static=True,
            bodyexclude=-1,
            geomid=np.zeros(1, dtype=np.int32),
        ),
        all_of(greater_than(0.0), less_than(3 * math.sqrt(2))),
        "a level ray from the middle did not meet a wall of the room",
    )


def test_workshop_starts_the_robot_clear_of_everything() -> None:
    """At the start the robot touches nothing but the floor."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "workshop.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
    mujoco.mj_forward(model, data)
    assert_that(
        [
            pair
            for pair in data.contact.geom.tolist()
            if model.geom("floor").id not in pair
        ],
        equal_to([]),
        "robot did not start clear of the walls and the furniture",
    )


def test_workshop_starts_the_robot_facing_north() -> None:
    """The start keyframe turns the robot towards the positive y axis."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "workshop.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
    mujoco.mj_forward(model, data)
    assert_that(
        math.atan2(
            data.body("base_link").xmat[3], data.body("base_link").xmat[0]
        ),
        close_to(math.pi / 2, 1e-6),
        "robot did not start facing north",
    )


def test_workshop_hides_the_pallet_under_the_lidar() -> None:
    """The lidar scan plane passes over the low pallet in front of the start."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "workshop.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("start").id)
    mujoco.mj_forward(model, data)
    assert_that(
        mujoco.mj_ray(
            model,
            data,
            data.body("base_scan").xpos,
            data.body("base_link").xmat.reshape(3, 3)[:, 0].copy(),
            np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8),
            flg_static=True,
            bodyexclude=-1,
            geomid=np.zeros(1, dtype=np.int32),
        ),
        greater_than(1.0),
        "lidar scan plane did not pass over the low pallet",
    )
