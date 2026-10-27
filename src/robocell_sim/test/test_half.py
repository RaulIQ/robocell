"""Tests for the pairing that waits for the accelerometer frame."""

import secrets

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.imu.half import Half
from robocell_sim.robot.implementations.imu.received import Received

from test.tallied import Tallied


def test_half_completes_with_the_accelerometer_frame_of_its_number() -> None:
    """A gyro and an accelerometer frame with one number make a pair."""
    number = secrets.SystemRandom().randint(0, 255)
    assert_that(
        Half(Received(0x101, bytes(6) + bytes((number, 0))), 0)
        .read(Received(0x102, bytes(6) + bytes((number, 0))))
        .complete(),
        "frames with the same number did not make a complete pair",
    )


def test_half_reports_a_second_gyro_frame_as_one_loss() -> None:
    """Two gyro frames in a row lose the measurement of the first one."""
    number = secrets.SystemRandom().randint(0, 255)
    counts: list[int] = []
    Half(Received(0x101, bytes(6) + bytes((number, 0))), 0).read(
        Received(0x101, bytes(6) + bytes(((number + 1) % 256, 0)))
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([1]),
        "second gyro frame did not lose one measurement",
    )


def test_half_reports_both_measurements_of_mismatched_frames() -> None:
    """Gyro 17 followed by accelerometer 18 loses both measurements."""
    number = secrets.SystemRandom().randint(0, 255)
    counts: list[int] = []
    Half(Received(0x101, bytes(6) + bytes((number, 0))), 0).read(
        Received(0x102, bytes(6) + bytes(((number + 1) % 256, 0)))
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([2]),
        "mismatched frames did not lose two measurements",
    )


def test_half_reports_the_gap_after_an_unanswered_gyro_frame() -> None:
    """A gyro frame further on loses this one and every one in between."""
    number = secrets.SystemRandom().randint(0, 255)
    skipped = secrets.SystemRandom().randint(0, 200)
    counts: list[int] = []
    Half(Received(0x101, bytes(6) + bytes((number, 0))), 0).read(
        Received(0x101, bytes(6) + bytes(((number + skipped + 1) % 256, 0)))
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([skipped + 1]),
        "gyro frame further on did not lose this and the skipped ones",
    )


def test_half_makes_no_pair_of_mismatched_frames() -> None:
    """An accelerometer frame with another number completes nothing."""
    number = secrets.SystemRandom().randint(0, 255)
    assert_that(
        not Half(Received(0x101, bytes(6) + bytes((number, 0))), 0)
        .read(Received(0x102, bytes(6) + bytes(((number + 1) % 256, 0))))
        .complete(),
        "mismatched frames did not stay an incomplete pair",
    )


def test_half_has_no_measurement() -> None:
    """Asking for the measurement with only the gyro frame fails at once."""
    with pytest.raises(ValueError, match="complete"):
        Half(Received(0x101, bytes(8)), 0).measurement()
