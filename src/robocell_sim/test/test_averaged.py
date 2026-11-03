"""Tests for online three-axis measurement statistics."""

from hamcrest import assert_that, close_to
from robocell_sim.robot.implementations.imu.averaged import Averaged


def test_averaged_measures_sample_variance_without_storing_the_window() -> None:
    """Two distinct observations establish the unbiased sample variance."""
    assert_that(
        Averaged((0, (0.0,) * 3, (0.0,) * 3))
        .read((0.003, 0.007, 0.004))
        .read((0.003, 0.007, 0.006))
        .variance()[2],
        close_to(2e-6, 1e-15),
        "online statistics did not recover the sample variance",
    )
