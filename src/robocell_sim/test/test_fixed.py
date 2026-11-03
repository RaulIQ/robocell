"""Tests for a gyro offset held fixed after calibration."""

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.imu.fixed import Fixed


def test_fixed_retains_the_offset_during_a_turn() -> None:
    """A learned offset cannot adapt to real angular motion."""
    assert_that(
        Fixed(((0.013, -0.007, 0.004), (1e-8,) * 3), 7.13, (2.0, 100, 0.003))
        .read((0.0, 0.0, 0.73), 7.17, still=False)
        .bias(),
        equal_to((0.013, -0.007, 0.004)),
        "turn did not retain the learned gyro offset",
    )
