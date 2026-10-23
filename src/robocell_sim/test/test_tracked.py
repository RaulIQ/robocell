"""Tests for the step that publishes the true pose of a body."""

import math
import secrets
from pathlib import Path
from typing import Any

import mujoco
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.hardware.implementations.physical import Physical
from robocell_sim.robot.implementations.differential import Differential
from robocell_sim.ros.implementations.tracked import Tracked

from test.kept import Kept
from test.waffle import Waffle


def test_tracked_publishes_the_true_position_of_the_body() -> None:
    """The odometry message puts the body where MuJoCo has it."""
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
    Tracked(Physical(model, data), data, "base_link", Kept(messages)).take()
    assert_that(
        math.dist(
            (
                messages[-1].pose.pose.position.x,
                messages[-1].pose.pose.position.y,
                messages[-1].pose.pose.position.z,
            ),
            data.body("base_link").xpos,
        ),
        close_to(0.0, 1e-9),
        "odometry message did not put the body where MuJoCo has it",
    )


def test_tracked_publishes_the_true_orientation_of_the_body() -> None:
    """The odometry message turns the body as MuJoCo has it turned."""
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
    Tracked(Physical(model, data), data, "base_link", Kept(messages)).take()
    assert_that(
        math.dist(
            (
                messages[-1].pose.pose.orientation.w,
                messages[-1].pose.pose.orientation.x,
                messages[-1].pose.pose.orientation.y,
                messages[-1].pose.pose.orientation.z,
            ),
            data.body("base_link").xquat,
        ),
        close_to(0.0, 1e-9),
        "odometry message did not turn the body as MuJoCo has it",
    )


def test_tracked_publishes_the_forward_speed_in_the_body_frame() -> None:
    """The linear twist is the commanded speed whatever the heading."""
    linear = secrets.SystemRandom().uniform(0.05, 0.25)
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = Waffle(
        model,
        *Differential(0.033, 0.288).wheels(linear, 0.0),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    messages: list[Any] = []
    Tracked(Physical(model, data), data, "base_link", Kept(messages)).take()
    assert_that(
        messages[-1].twist.twist.linear.x,
        close_to(linear, linear * 0.1),
        "odometry message did not carry the forward speed in the body frame",
    )


def test_tracked_publishes_the_yaw_rate_of_the_body() -> None:
    """The angular twist around the vertical axis is the commanded turn."""
    angular = secrets.SystemRandom().uniform(0.3, 1.0)
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = Waffle(
        model,
        *Differential(0.033, 0.288).wheels(
            secrets.SystemRandom().uniform(0.05, 0.1), angular
        ),
        secrets.SystemRandom().uniform(-math.pi, math.pi),
    ).state()
    messages: list[Any] = []
    Tracked(Physical(model, data), data, "base_link", Kept(messages)).take()
    assert_that(
        messages[-1].twist.twist.angular.z,
        close_to(angular, angular * 0.1),
        "odometry message did not carry the yaw rate of the body",
    )


def test_tracked_stamps_the_pose_with_the_time_of_the_simulation() -> None:
    """The header carries the simulated time after the step."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    count = secrets.SystemRandom().randint(2, 900)
    for _ in range(count):
        Tracked(Physical(model, data), data, "base_link", Kept(messages)).take()
    assert_that(
        messages[-1].header.stamp.sec
        + messages[-1].header.stamp.nanosec * 1e-9,
        close_to(count * model.opt.timestep, 1e-9),
        "odometry message was not stamped with the time of the simulation",
    )


def test_tracked_names_the_body_as_the_child_frame() -> None:
    """The child frame of the odometry is the tracked body."""
    model = mujoco.MjModel.from_xml_path(
        str(Path(__file__).parents[1] / "models" / "waffle_pi" / "scene.xml")
    )
    data = mujoco.MjData(model)
    messages: list[Any] = []
    body = secrets.choice(("base_link", "wheel_left_link", "wheel_right_link"))
    Tracked(Physical(model, data), data, body, Kept(messages)).take()
    assert_that(
        messages[-1].child_frame_id,
        equal_to(body),
        "odometry message did not name the body as its child frame",
    )
