"""Tests for the report that publishes an IMU reading."""

import math
import secrets
import struct
from typing import Any

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.imu.received import Received
from robocell_sim.robot.implementations.imu.scaled import Scaled
from robocell_sim.robot.implementations.imu.whole import Whole
from robocell_sim.ros.implementations.imu.inertial import Inertial
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.kept import Kept


def test_inertial_keeps_the_frame_of_the_header() -> None:
    """The message lies in the frame its header names."""
    frame = f"imu_{secrets.token_hex(4)}"
    messages: list[Any] = []
    Inertial(
        Scaled(Whole(Received(0x101, bytes(8)), Received(0x102, bytes(8)))),
        Header(frame_id=frame),
        (1e-6, 4e-4),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].header.frame_id,
        equal_to(frame),
        "message did not lie in the frame of its header",
    )


def test_inertial_marks_the_orientation_as_unknown() -> None:
    """The board sends no orientation, so its covariance starts with -1."""
    messages: list[Any] = []
    Inertial(
        Scaled(Whole(Received(0x101, bytes(8)), Received(0x102, bytes(8)))),
        Header(frame_id="imu_link"),
        (1e-6, 4e-4),
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].orientation_covariance[0],
        equal_to(-1.0),
        "message did not mark the orientation as unknown",
    )


def test_inertial_carries_the_scaled_angular_velocity() -> None:
    """The angular velocity is the rate of the reading in rad/s."""
    counts = (
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
    )
    messages: list[Any] = []
    Inertial(
        Scaled(
            Whole(
                Received(0x101, struct.pack("<hhhBx", *counts, 3)),
                Received(0x102, bytes(6) + bytes((3, 0))),
            )
        ),
        Header(frame_id="imu_link"),
        (1e-6, 4e-4),
        Kept(messages),
    ).publish()
    assert_that(
        math.dist(
            (
                messages[-1].angular_velocity.x,
                messages[-1].angular_velocity.y,
                messages[-1].angular_velocity.z,
            ),
            tuple(count / 1000 for count in counts),
        ),
        close_to(0.0, 1e-9),
        "message did not carry the scaled angular velocity",
    )


def test_inertial_carries_the_scaled_linear_acceleration() -> None:
    """The linear acceleration is the acceleration of the reading in m/s²."""
    counts = (
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
    )
    messages: list[Any] = []
    Inertial(
        Scaled(
            Whole(
                Received(0x101, bytes(6) + bytes((5, 0))),
                Received(0x102, struct.pack("<hhhBx", *counts, 5)),
            )
        ),
        Header(frame_id="imu_link"),
        (1e-6, 4e-4),
        Kept(messages),
    ).publish()
    assert_that(
        math.dist(
            (
                messages[-1].linear_acceleration.x,
                messages[-1].linear_acceleration.y,
                messages[-1].linear_acceleration.z,
            ),
            tuple(count / 1000 for count in counts),
        ),
        close_to(0.0, 1e-9),
        "message did not carry the scaled linear acceleration",
    )


def test_inertial_puts_the_variances_on_the_diagonals() -> None:
    """Each covariance holds its variance on the diagonal and zero elsewhere."""
    gyro = secrets.SystemRandom().uniform(1e-8, 1e-4)
    accelerometer = secrets.SystemRandom().uniform(1e-6, 1e-2)
    messages: list[Any] = []
    Inertial(
        Scaled(Whole(Received(0x101, bytes(8)), Received(0x102, bytes(8)))),
        Header(frame_id="imu_link"),
        (gyro, accelerometer),
        Kept(messages),
    ).publish()
    assert_that(
        (
            list(messages[-1].angular_velocity_covariance),
            list(messages[-1].linear_acceleration_covariance),
        ),
        equal_to(
            (
                [gyro, 0.0, 0.0, 0.0, gyro, 0.0, 0.0, 0.0, gyro],
                [
                    accelerometer,
                    0.0,
                    0.0,
                    0.0,
                    accelerometer,
                    0.0,
                    0.0,
                    0.0,
                    accelerometer,
                ],
            )
        ),
        "message did not put the variances on the covariance diagonals",
    )
