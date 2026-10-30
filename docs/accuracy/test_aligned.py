"""Tests for time and frame alignment of odometry errors."""

import math

from hamcrest import assert_that, close_to, contains_exactly, equal_to

from docs.accuracy.aligned import Aligned


def test_alignment_transforms_odom_into_world_before_comparison() -> None:
    """A rotated and translated odom pose matches the same world pose."""
    assert_that(
        Aligned(
            ((0.0, 2.0, 4.0, math.pi / 2, 0.17, -0.3),),
            ((0.0, 1.0, 0.0, 0.0, 0.17, -0.3),),
            (2.0, 3.0, math.pi / 2),
        ).rows(),
        equal_to(((0.0, 0.0, 0.0, 0.0, 0.0),)),
        "alignment did not transform odom poses into the world frame",
    )


def test_alignment_interpolates_heading_across_the_angle_boundary() -> None:
    """Interpolation crosses pi by the short arc instead of through zero."""
    assert_that(
        Aligned(
            (
                (0.0, 0.0, 0.0, math.radians(179), 0.0, 0.0),
                (2.0, 0.0, 0.0, math.radians(-179), 0.0, 0.0),
            ),
            ((1.0, 0.0, 0.0, math.pi, 0.0, 0.0),),
            (0.0, 0.0, 0.0),
        ).rows()[0][2],
        close_to(0.0, 1e-12),
        "alignment did not interpolate heading across the angle boundary",
    )


def test_alignment_does_not_extrapolate_beyond_ground_truth() -> None:
    """Only estimates bracketed by recorded ground truth are compared."""
    assert_that(
        Aligned(
            ((1.0, 0.0, 0.0, 0.0, 0.0, 0.0),),
            (
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                (2.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            ),
            (0.0, 0.0, 0.0),
        ).rows(),
        equal_to(((1.0, 0.0, 0.0, 0.0, 0.0),)),
        "alignment did not discard estimates outside the reference interval",
    )


def test_alignment_compares_velocities_at_the_estimate_timestamp() -> None:
    """Interpolated body velocities retain signed residuals for bias checks."""
    assert_that(
        Aligned(
            (
                (0.0, 0.0, 0.0, 0.0, 0.1, -0.3),
                (2.0, 0.0, 0.0, 0.0, 0.3, -0.1),
            ),
            ((1.0, 0.0, 0.0, 0.0, 0.25, -0.25),),
            (0.0, 0.0, 0.0),
        ).rows()[0][3:],
        contains_exactly(close_to(0.05, 1e-12), close_to(-0.05, 1e-12)),
        "alignment did not compare signed body rates at the estimate time",
    )
