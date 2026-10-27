"""Tests for the pairing between two measurements of the IMU board."""

import secrets

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.imu.received import Received
from robocell_sim.robot.implementations.imu.waiting import Waiting

from test.tallied import Tallied


def test_waiting_reports_the_measurements_skipped_in_between() -> None:
    """A gyro frame numbered past the next one loses the skipped ones."""
    last = secrets.SystemRandom().randint(0, 255)
    skipped = secrets.SystemRandom().randint(0, 200)
    counts: list[int] = []
    Waiting(last, 0).read(
        Received(0x101, bytes(6) + bytes(((last + skipped + 1) % 256, 0)))
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([skipped]),
        "pairing did not report the measurements skipped in between",
    )


def test_waiting_reports_no_loss_when_the_number_wraps() -> None:
    """Measurement 0 after measurement 255 follows it without a gap."""
    counts: list[int] = []
    Waiting(255, 0).read(Received(0x101, bytes(8))).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([0]),
        "pairing did not let the number wrap from 255 to 0",
    )


def test_waiting_reports_a_lone_accelerometer_frame_as_lost() -> None:
    """An accelerometer frame with no gyro frame before it loses one."""
    last = secrets.SystemRandom().randint(0, 255)
    counts: list[int] = []
    Waiting(last, 0).read(
        Received(0x102, bytes(6) + bytes(((last + 1) % 256, 0)))
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([1]),
        "lone accelerometer frame did not lose one measurement",
    )


def test_waiting_has_no_measurement() -> None:
    """Asking for the measurement between measurements fails at once."""
    with pytest.raises(ValueError, match="complete"):
        Waiting(secrets.SystemRandom().randint(0, 255), 0).measurement()
