"""Tests for choosing an approach point before planning to an obstacle."""

from math import dist

from hamcrest import assert_that, close_to, equal_to

from robocell_sim.approach import approach


def test_approach_stays_a_meter_before_the_target() -> None:
    """The endpoint is separated from the object's occupied cell."""
    assert_that(
        approach((0.0, -1.6), (0.0, 0.0), 1.0),
        equal_to((0.0, -0.6000000000000001)),
        "the approach point did not stop before the target",
    )


def test_approach_doesnt_reverse_when_the_target_is_close() -> None:
    """A target already inside the following distance leaves the robot still."""
    assert_that(
        approach((0.17, -0.23), (0.31, -0.09), 0.8),
        equal_to((0.31, -0.09)),
        "a nearby target did not leave the robot at its current position",
    )


def test_approach_handles_a_target_at_the_robot_position() -> None:
    """Coincident coordinates require no division or motion."""
    assert_that(
        approach((0.37, -0.19), (0.37, -0.19), 0.9),
        equal_to((0.37, -0.19)),
        "a coincident target did not leave the robot still",
    )


def test_approach_preserves_the_requested_distance_on_a_diagonal() -> None:
    """The offset follows the target bearing rather than either map axis."""
    assert_that(
        dist(approach((2.7, -3.9), (-0.3, 0.1), 1.3), (2.7, -3.9)),
        close_to(1.3, 1e-10),
        "the diagonal approach did not preserve the following distance",
    )
