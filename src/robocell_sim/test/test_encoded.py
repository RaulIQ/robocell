"""Tests for the standard five-byte RPLIDAR measurement."""

import math

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.lidar.encoded import Encoded


@pytest.mark.parametrize(
    ("measurement", "payload"),
    [
        ((0.0, 1.0, True, 15), "3D 01 00 A0 0F"),
        ((1.0, 1.0, False, 15), "3E 81 00 A0 0F"),
        ((90.0, 1.0, False, 15), "3E 01 2D A0 0F"),
        ((359.0, 3.5, False, 15), "3E 81 B3 B0 36"),
        ((0.0, math.inf, True, 15), "01 01 00 00 00"),
    ],
)
def test_encoded_matches_the_protocol_examples(
    measurement: tuple[float, float, bool, int], payload: str
) -> None:
    """The byte examples in the wire contract encode exactly."""
    assert_that(
        Encoded(*measurement).encode(),
        equal_to(bytes.fromhex(payload)),
        "measurement did not match the documented wire bytes",
    )


@pytest.mark.parametrize("distance", [math.nan, -1.7, 0.0, 16.384])
def test_encoded_marks_unrepresentable_returns_invalid(distance: float) -> None:
    """Invalid distances cannot wrap into a plausible obstacle."""
    assert_that(
        Encoded(37.25, distance, _start=False, _quality=31).encode(),
        equal_to(bytes.fromhex("02 A1 12 00 00")),
        "invalid return did not clear distance and quality",
    )
