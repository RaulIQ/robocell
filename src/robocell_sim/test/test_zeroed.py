"""Tests for stationary gyro offset estimation."""

import math
from typing import TYPE_CHECKING

from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.imu.averaged import Averaged
from robocell_sim.robot.implementations.imu.zeroed import Zeroed

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.zero import Zero


def test_zeroed_removes_the_offset_of_a_stationary_gyro() -> None:
    """A stationary window identifies the three constant offsets."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    assert_that(
        math.dist(zero.bias(), (0.0037, -0.0081, 0.0049)),
        close_to(0.0, 1e-12),
        "stationary measurements did not recover the gyro offset",
    )


def test_zeroed_waits_for_a_complete_stationary_window() -> None:
    """A short stationary interval cannot establish the offset."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(50):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    assert_that(
        zero.ready(),
        equal_to(obj=False),
        "short interval did not withhold calibration",
    )


def test_zeroed_does_not_learn_a_turn_after_calibration() -> None:
    """Motion after calibration cannot change the learned offset."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    assert_that(
        zero.read((0.31, -0.17, 0.83), 8.15, still=False).bias(),
        equal_to(zero.bias()),
        "motion did not preserve the calibrated offset",
    )


def test_zeroed_restarts_the_window_when_the_robot_moves() -> None:
    """Movement prevents combining separate stationary intervals."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(90):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    zero = zero.read((0.0037, -0.0081, 0.0049), 8.04, still=False)
    for index in range(30):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 8.05 + index / 100, still=True
        )
    assert_that(
        zero.ready(),
        equal_to(obj=False),
        "movement did not restart the stationary window",
    )


def test_zeroed_restarts_after_a_measurement_gap() -> None:
    """Missing measurements cannot count as observed stationary time."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(90):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    assert_that(
        zero.read((0.0037, -0.0081, 0.0049), 8.67, still=True).ready(),
        equal_to(obj=False),
        "measurement gap did not restart the stationary window",
    )


def test_zeroed_discards_calibration_when_time_moves_backwards() -> None:
    """A new simulation epoch cannot inherit the old sensor offset."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    assert_that(
        zero.read((0.017, -0.003, -0.011), 0.13, still=True).ready(),
        equal_to(obj=False),
        "backwards time did not discard the old calibration",
    )


def test_zeroed_does_not_count_duplicate_timestamps() -> None:
    """A paused clock cannot supply a calibration interval."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for _ in range(101):
        zero = zero.read((0.0037, -0.0081, 0.0049), 7.13, still=True)
    assert_that(
        zero.ready(),
        equal_to(obj=False),
        "duplicate time did not withhold calibration",
    )


def test_zeroed_rejects_an_unstable_stationary_window() -> None:
    """Wheel standstill cannot turn a varying gyro rate into an offset."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0, 0.0, 0.01 * (-1) ** index), 7.13 + index / 100, still=True
        )
    assert_that(
        zero.ready(),
        equal_to(obj=False),
        "unstable measurements did not withhold calibration",
    )


def test_zeroed_reports_the_uncertainty_of_its_mean() -> None:
    """Offset uncertainty uses the variance of the calibration mean."""
    zero = (
        Zeroed(
            (0.013, 2, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
        )
        .read((0.0, 0.0, 0.004), 7.13, still=True)
        .read((0.0, 0.0, 0.006), 7.15, still=True)
    )
    assert_that(
        zero.variance()[2],
        close_to(1e-6, 1e-15),
        "calibration did not report the uncertainty of its mean",
    )


def test_zeroed_rejects_large_rotation_even_when_the_wheels_stand_still() -> (
    None
):
    """Strong gyro rotation cannot become a stationary offset."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read((0.0, 0.0, 0.37), 7.13 + index / 100, still=True)
    assert_that(
        zero.ready(),
        equal_to(obj=False),
        "strong rotation did not withhold calibration",
    )


def test_zeroed_recovers_after_the_robot_stops() -> None:
    """A fresh stationary interval permits calibration after motion."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read((0.0, 0.0, 0.37), 7.13 + index / 100, still=False)
    for index in range(101):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 8.15 + index / 100, still=True
        )
    assert_that(
        math.dist(zero.bias(), (0.0037, -0.0081, 0.0049)),
        close_to(0.0, 1e-12),
        "new stationary interval did not recover calibration after motion",
    )


def test_zeroed_replaces_an_unstable_window_with_fresh_measurements() -> None:
    """Old unstable samples cannot contaminate the next calibration window."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(99):
        zero = zero.read(
            (0.0, 0.0, 0.004 * (-1) ** index), 7.13 + index / 100, still=True
        )
    for index in range(101):
        zero = zero.read((0.0, 0.0, 0.001), 8.13 + index / 100, still=True)
    assert_that(
        zero.bias()[2],
        close_to(0.001, 1e-12),
        "unstable samples did not leave the next calibration window",
    )
