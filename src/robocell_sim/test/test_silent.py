"""Tests for the pairing before the first frame of the IMU board."""

import secrets

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.imu.received import Received
from robocell_sim.robot.implementations.imu.silent import Silent

from test.tallied import Tallied


def test_silent_reports_no_loss_for_the_first_frame() -> None:
    """The first gyro frame loses nothing, whatever its number."""
    counts: list[int] = []
    Silent(0).read(
        Received(
            0x101,
            bytes(6) + bytes((secrets.SystemRandom().randint(0, 255), 0)),
        )
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([0]),
        "first frame did not pass without a loss",
    )


def test_silent_makes_no_pair_of_a_lone_gyro_frame() -> None:
    """A gyro frame alone does not complete a measurement."""
    assert_that(
        not Silent(0)
        .read(
            Received(
                0x101,
                bytes(6) + bytes((secrets.SystemRandom().randint(0, 255), 0)),
            )
        )
        .complete(),
        "lone gyro frame did not stay an incomplete pair",
    )


def test_silent_reports_a_lone_accelerometer_frame_as_lost() -> None:
    """An accelerometer frame with no gyro frame before it loses one."""
    counts: list[int] = []
    Silent(0).read(
        Received(
            0x102,
            bytes(6) + bytes((secrets.SystemRandom().randint(0, 255), 0)),
        )
    ).note(Tallied(counts))
    assert_that(
        counts,
        equal_to([1]),
        "lone accelerometer frame did not lose one measurement",
    )


def test_silent_has_no_measurement() -> None:
    """Asking for the measurement before any frame fails at once."""
    with pytest.raises(ValueError, match="complete"):
        Silent(0).measurement()
