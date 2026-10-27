"""Tests for the reading that scales a complete pair to SI units."""

import math
import secrets
import struct

from hamcrest import assert_that, close_to
from robocell_sim.robot.implementations.imu.received import Received
from robocell_sim.robot.implementations.imu.scaled import Scaled
from robocell_sim.robot.implementations.imu.whole import Whole


def test_scaled_turns_a_thousand_gyro_counts_into_one_radian() -> None:
    """Raw 1000 in the gyro frame is 1.0 rad/s."""
    axis = secrets.SystemRandom().randint(0, 2)
    assert_that(
        Scaled(
            Whole(
                Received(
                    0x101,
                    struct.pack(
                        "<hhhBx", *(1000 * (i == axis) for i in range(3)), 4
                    ),
                ),
                Received(0x102, struct.pack("<hhhBx", 0, 0, 0, 4)),
            )
        ).rate()[axis],
        close_to(1.0, 1e-12),
        "thousand gyro counts did not make one radian per second",
    )


def test_scaled_turns_a_thousand_accelerometer_counts_into_one_meter() -> None:
    """Raw 1000 in the accelerometer frame is 1.0 m/s²."""
    axis = secrets.SystemRandom().randint(0, 2)
    assert_that(
        Scaled(
            Whole(
                Received(0x101, struct.pack("<hhhBx", 0, 0, 0, 4)),
                Received(
                    0x102,
                    struct.pack(
                        "<hhhBx", *(1000 * (i == axis) for i in range(3)), 4
                    ),
                ),
            )
        ).acceleration()[axis],
        close_to(1.0, 1e-12),
        "thousand accelerometer counts did not make one meter per second²",
    )


def test_scaled_turns_minus_one_count_into_minus_a_thousandth() -> None:
    """Raw -1 is -0.001 in every axis of both sensors."""
    assert_that(
        math.dist(
            Scaled(
                Whole(
                    Received(0x101, struct.pack("<hhhBx", -1, -1, -1, 4)),
                    Received(0x102, struct.pack("<hhhBx", -1, -1, -1, 4)),
                )
            ).rate()
            + Scaled(
                Whole(
                    Received(0x101, struct.pack("<hhhBx", -1, -1, -1, 4)),
                    Received(0x102, struct.pack("<hhhBx", -1, -1, -1, 4)),
                )
            ).acceleration(),
            (-0.001,) * 6,
        ),
        close_to(0.0, 1e-12),
        "minus one count did not make minus a thousandth",
    )
