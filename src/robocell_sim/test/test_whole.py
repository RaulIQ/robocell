"""Tests for the pairing that holds both frames of one measurement."""

import secrets
import struct

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.imu.received import Received
from robocell_sim.robot.implementations.imu.whole import Whole

from test.tallied import Tallied


def test_whole_gives_the_raw_values_of_both_frames() -> None:
    """The measurement is the gyro x, y, z and then the accelerometer ones."""
    gyro = (
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
    )
    accelerometer = (
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
        secrets.SystemRandom().randint(-32768, 32767),
    )
    assert_that(
        Whole(
            Received(0x101, struct.pack("<hhhBx", *gyro, 9)),
            Received(0x102, struct.pack("<hhhBx", *accelerometer, 9)),
        ).measurement(),
        equal_to((gyro, accelerometer)),
        "complete pair did not give the raw values of both frames",
    )


def test_whole_is_complete() -> None:
    """A pairing that holds both frames is complete."""
    number = secrets.SystemRandom().randint(0, 255)
    assert_that(
        Whole(
            Received(0x101, bytes(6) + bytes((number, 0))),
            Received(0x102, bytes(6) + bytes((number, 0))),
        ).complete(),
        "pairing with both frames was not complete",
    )


def test_whole_reports_no_loss_for_the_frame_that_completes_it() -> None:
    """The accelerometer frame that completes a pair loses nothing."""
    number = secrets.SystemRandom().randint(0, 255)
    counts: list[int] = []
    Whole(
        Received(0x101, bytes(6) + bytes((number, 0))),
        Received(0x102, bytes(6) + bytes((number, 0))),
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([0]),
        "frame that completed the pair did not pass without a loss",
    )


def test_whole_reports_no_loss_for_the_next_measurement() -> None:
    """The gyro frame of the next number follows a pair without a loss."""
    number = secrets.SystemRandom().randint(0, 255)
    counts: list[int] = []
    Whole(
        Received(0x101, bytes(6) + bytes((number, 0))),
        Received(0x102, bytes(6) + bytes((number, 0))),
    ).read(Received(0x101, bytes(6) + bytes(((number + 1) % 256, 0)))).note(
        Tallied(counts)
    )
    assert_that(
        counts,
        equal_to([0]),
        "next measurement did not follow the pair without a loss",
    )
