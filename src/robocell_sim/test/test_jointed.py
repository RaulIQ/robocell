"""Tests for the report that publishes the wheel joints by their encoders."""

import math
import secrets
from pathlib import Path
from typing import Any

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.simulated import Simulated
from robocell_sim.ros.implementations.jointed import Jointed

from test.kept import Kept
from test.waffle import Waffle


def test_jointed_names_the_wheel_joints_of_the_urdf() -> None:
    """The joint state names the wheels as the URDF does, in given order."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    Jointed(
        data,
        {
            "wheel_left_joint": Simulated(data, "wheel_left_encoder", 4096),
            "wheel_right_joint": Simulated(data, "wheel_right_encoder", 4096),
        },
        Kept(messages),
    ).publish()
    assert_that(
        list(messages[-1].name),
        equal_to(["wheel_left_joint", "wheel_right_joint"]),
        "joint state did not name the wheel joints of the URDF",
    )


def test_jointed_publishes_the_angles_the_encoders_read() -> None:
    """The joint positions are the encoder angles, not the exact MuJoCo ones."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = Waffle(
        model,
        secrets.SystemRandom().uniform(-6.0, 6.0),
        secrets.SystemRandom().uniform(-6.0, 6.0),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    resolution = secrets.SystemRandom().randint(64, 512)
    messages: list[Any] = []
    Jointed(
        data,
        {
            "wheel_left_joint": Simulated(
                data, "wheel_left_encoder", resolution
            ),
            "wheel_right_joint": Simulated(
                data, "wheel_right_encoder", resolution
            ),
        },
        Kept(messages),
    ).publish()
    assert_that(
        math.dist(
            messages[-1].position,
            (
                Simulated(data, "wheel_left_encoder", resolution).angle(),
                Simulated(data, "wheel_right_encoder", resolution).angle(),
            ),
        ),
        close_to(0.0, 1e-9),
        "joint state did not carry the angles the encoders read",
    )


def test_jointed_stamps_the_joints_with_the_simulated_time() -> None:
    """The joint state carries the time of the simulation it publishes."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    mujoco.mj_step(model, data, nstep=secrets.SystemRandom().randint(2, 900))
    Jointed(
        data,
        {"wheel_left_joint": Simulated(data, "wheel_left_encoder", 4096)},
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].header.stamp.sec
        + messages[-1].header.stamp.nanosec * 1e-9,
        close_to(data.time, 1e-9),
        "joint state did not carry the simulated time",
    )
