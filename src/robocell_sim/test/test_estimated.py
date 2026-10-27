"""Tests for the report that publishes the odometry of the robot."""

import math
import secrets
from typing import Any

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.unread import Unread
from robocell_sim.ros.implementations.odometry.estimated import Estimated
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.kept import Kept


def test_estimated_names_the_body_it_tracks() -> None:
    """The odometry message names the body frame it places."""
    child = f"base_{secrets.token_hex(4)}"
    messages: list[Any] = []
    Estimated(
        Unread(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0)),
        Header(frame_id="odom"),
        child,
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].child_frame_id,
        equal_to(child),
        "odometry message did not name the body it tracks",
    )


def test_estimated_carries_the_position_of_the_odometry() -> None:
    """The pose holds x and y as the odometry reckons them, on the floor."""
    pose = (
        secrets.SystemRandom().uniform(-3.0, 3.0),
        secrets.SystemRandom().uniform(-3.0, 3.0),
        0.0,
    )
    messages: list[Any] = []
    Estimated(
        Unread(Differential(0.033, 0.288), 4096, pose),
        Header(frame_id="odom"),
        "base_footprint",
        Kept(messages),
    ).publish()
    assert_that(
        (
            messages[-1].pose.pose.position.x,
            messages[-1].pose.pose.position.y,
            messages[-1].pose.pose.position.z,
        ),
        equal_to(pose),
        "odometry message did not carry the reckoned position",
    )


def test_estimated_turns_the_pose_to_the_heading() -> None:
    """The orientation is a turn about z by the reckoned heading."""
    heading = secrets.SystemRandom().uniform(-math.pi, math.pi)
    messages: list[Any] = []
    Estimated(
        Unread(Differential(0.033, 0.288), 4096, (0.0, 0.0, heading)),
        Header(frame_id="odom"),
        "base_footprint",
        Kept(messages),
    ).publish()
    assert_that(
        2
        * math.atan2(
            messages[-1].pose.pose.orientation.z,
            messages[-1].pose.pose.orientation.w,
        ),
        close_to(heading, 1e-9),
        "odometry message did not turn the pose to the heading",
    )
