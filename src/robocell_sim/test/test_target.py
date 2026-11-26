"""Tests for displaying the actual object's position rather than an approach goal."""

from math import dist

from hamcrest import assert_that, close_to
from robocell_sim.ros.implementations.pose.target import marker, pose


def test_target_marker_moves_by_the_same_distance_as_the_object() -> None:
    """The marker preserves object motion in the stationary odometry frame."""
    before = marker(pose((-0.53, -2.41, 0.7), (-2.13, -2.57, 1.57)))
    after = marker(pose((-0.29, -2.09, 0.9), (-2.13, -2.57, 1.57)))
    assert_that(
        dist(
            (before.pose.position.x, before.pose.position.y),
            (after.pose.position.x, after.pose.position.y),
        ),
        close_to(0.4, 1e-10),
        "the displayed marker did not preserve the object's displacement",
    )
