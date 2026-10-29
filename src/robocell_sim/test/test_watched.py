"""Tests for serial liveness with simulated-time pauses."""

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.lidar.watched import Watched


def test_watched_does_not_time_out_a_paused_clock() -> None:
    """A frozen device clock cannot look like an active but silent sensor."""
    assert_that(
        Watched((0.0, 100, 0.0), 2.0).read(b"", 100, 20.0),
        equal_to((0.0, 100, 0.0)),
        "paused clock did not preserve the waiting session",
    )


def test_watched_gives_a_resumed_sensor_time_to_produce_new_data() -> None:
    """A resumed clock starts a fresh grace period before UART recovery."""
    assert_that(
        Watched((0.0, 100, 0.0), 2.0).read(b"", 101, 20.0),
        equal_to((20.0, 101, 20.0)),
        "resumed sensor did not receive a fresh grace period",
    )


def test_watched_times_out_a_silent_sensor_with_an_advancing_clock() -> None:
    """An active clock cannot excuse a device that has stopped sending data."""
    with pytest.raises(TimeoutError) as failure:
        Watched((0.0, 100, 2.0), 2.0).read(b"", 101, 2.1)
    assert_that(
        str(failure.value),
        equal_to("RPLIDAR measurements timed out while ROS time advanced"),
        "silent device did not time out while its clock advanced",
    )


def test_watched_restarts_after_a_clock_reset() -> None:
    """Old acquisition state cannot survive a backwards simulated clock."""
    with pytest.raises(ValueError, match="clock moved backwards") as failure:
        Watched((1.0, 100, 1.0), 2.0).read(b"", 99, 1.1)
    assert_that(
        str(failure.value),
        equal_to("RPLIDAR ROS clock moved backwards, so restart acquisition"),
        "backwards clock did not invalidate the serial session",
    )
