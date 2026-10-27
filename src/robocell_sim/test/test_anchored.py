"""Tests for the report that places the body by the odometry in TF."""

import math
import secrets
from typing import Any

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.unread import Unread
from robocell_sim.ros.implementations.odometry.anchored import Anchored
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.kept import Kept


def test_anchored_links_the_body_to_the_header_frame() -> None:
    """The transform runs from the frame of the header to the body."""
    child = f"base_{secrets.token_hex(4)}"
    messages: list[Any] = []
    Anchored(
        Unread(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0)),
        Header(frame_id="odom"),
        child,
        Kept(messages),
    ).publish()
    assert_that(
        (
            messages[-1].transforms[0].header.frame_id,
            messages[-1].transforms[0].child_frame_id,
        ),
        equal_to(("odom", child)),
        "transform did not link the body to the frame of the header",
    )


def test_anchored_moves_the_body_to_the_reckoned_position() -> None:
    """The translation is x and y of the odometry, on the floor."""
    pose = (
        secrets.SystemRandom().uniform(-3.0, 3.0),
        secrets.SystemRandom().uniform(-3.0, 3.0),
        0.0,
    )
    messages: list[Any] = []
    Anchored(
        Unread(Differential(0.033, 0.288), 4096, pose),
        Header(frame_id="odom"),
        "base_footprint",
        Kept(messages),
    ).publish()
    assert_that(
        (
            messages[-1].transforms[0].transform.translation.x,
            messages[-1].transforms[0].transform.translation.y,
            messages[-1].transforms[0].transform.translation.z,
        ),
        equal_to(pose),
        "transform did not move the body to the reckoned position",
    )


def test_anchored_turns_the_body_to_the_heading() -> None:
    """The rotation is a turn about z by the reckoned heading."""
    heading = secrets.SystemRandom().uniform(-math.pi, math.pi)
    messages: list[Any] = []
    Anchored(
        Unread(Differential(0.033, 0.288), 4096, (0.0, 0.0, heading)),
        Header(frame_id="odom"),
        "base_footprint",
        Kept(messages),
    ).publish()
    assert_that(
        2
        * math.atan2(
            messages[-1].transforms[0].transform.rotation.z,
            messages[-1].transforms[0].transform.rotation.w,
        ),
        close_to(heading, 1e-9),
        "transform did not turn the body to the heading",
    )
