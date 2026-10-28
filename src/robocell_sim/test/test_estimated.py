"""Tests for the report that publishes the odometry of the robot."""

import math
import secrets
from typing import Any

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.drive.differential import Differential
from robocell_sim.robot.implementations.odometry.parked import Parked
from robocell_sim.robot.implementations.odometry.rolling import Rolling
from robocell_sim.ros.implementations.odometry.estimated import Estimated
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.kept import Kept


def test_estimated_names_the_body_it_tracks() -> None:
    """The odometry message names the body frame it places."""
    child = f"base_{secrets.token_hex(4)}"
    messages: list[Any] = []
    Estimated(
        Parked(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0)),
        (1e-4, 4e-4),
        (Header(frame_id="odom"), child),
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
        Parked(Differential(0.033, 0.288), 4096, pose),
        (1e-4, 4e-4),
        (Header(frame_id="odom"), "base_footprint"),
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
        Parked(Differential(0.033, 0.288), 4096, (0.0, 0.0, heading)),
        (1e-4, 4e-4),
        (Header(frame_id="odom"), "base_footprint"),
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


def test_estimated_carries_the_forward_speed() -> None:
    """The twist moves forward along x at the speed the wheels give."""
    count = secrets.SystemRandom().randint(1, 5000)
    messages: list[Any] = []
    travel = Rolling(
        Differential(0.033, 0.288),
        4096,
        (0.0, 0.0, 0.0),
        ((0, 0, 1.0), (count, count, 1.02)),
    )
    Estimated(
        travel,
        (1e-4, 4e-4),
        (Header(frame_id="odom"), "base_footprint"),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].twist.twist.linear.x,
        close_to(travel.velocity()[0], 1e-12),
        "odometry message did not carry the forward speed",
    )


def test_estimated_carries_the_yaw_rate() -> None:
    """The twist turns about z at the rate the wheels give."""
    count = secrets.SystemRandom().randint(1, 2000)
    messages: list[Any] = []
    travel = Rolling(
        Differential(0.033, 0.288),
        4096,
        (0.0, 0.0, 0.0),
        ((0, 0, 1.0), (-count, count, 1.02)),
    )
    Estimated(
        travel,
        (1e-4, 4e-4),
        (Header(frame_id="odom"), "base_footprint"),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].twist.twist.angular.z,
        close_to(travel.velocity()[1], 1e-12),
        "odometry message did not carry the yaw rate",
    )


def test_estimated_spreads_the_forward_speed_by_its_variance() -> None:
    """The covariance of the forward speed is the variance it was given."""
    variance = secrets.SystemRandom().uniform(1e-6, 1e-2)
    messages: list[Any] = []
    Estimated(
        Parked(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0)),
        (variance, 4e-4),
        (Header(frame_id="odom"), "base_footprint"),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].twist.covariance[0],
        equal_to(variance),
        "odometry message did not spread the forward speed by its variance",
    )


def test_estimated_spreads_the_yaw_rate_by_its_variance() -> None:
    """The covariance of the yaw rate is the variance it was given."""
    variance = secrets.SystemRandom().uniform(1e-6, 1e-2)
    messages: list[Any] = []
    Estimated(
        Parked(Differential(0.033, 0.288), 4096, (0.0, 0.0, 0.0)),
        (1e-4, variance),
        (Header(frame_id="odom"), "base_footprint"),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].twist.covariance[35],
        equal_to(variance),
        "odometry message did not spread the yaw rate by its variance",
    )
