"""Tests for detecting an odometry discrepancy and confirming recovery."""

from hamcrest import assert_that, equal_to
from robocell_sim.recovery import blocked, confirmed


def test_unchanged_ranges_disagree_with_wheel_travel() -> None:
    assert_that(
        blocked([1.4] * 60, [1.405] * 60, 0.14),
        equal_to(True),
        "stationary ranges did not contradict wheel travel",
    )


def test_changed_ranges_allow_mapping_to_continue() -> None:
    assert_that(
        blocked([1.4] * 60, [1.52] * 60, 0.14),
        equal_to(False),
        "consistent range changes stopped mapping",
    )


def test_invalid_ranges_cannot_confirm_slip() -> None:
    assert_that(
        blocked([float("inf")] * 60, [float("inf")] * 60, 0.14),
        equal_to(False),
        "invalid ranges were accepted as evidence of slip",
    )


def test_consecutive_corrections_must_agree() -> None:
    assert_that(
        confirmed((0.8, 0.02, 0.01), [(0.81, 0, 0.02), (0.4, 0, 0)]),
        equal_to(False),
        "an inconsistent correction resumed mapping",
    )


def test_three_agreeing_corrections_can_resume_mapping() -> None:
    assert_that(
        confirmed((0.8, 0.02, 0.01), [(0.81, 0, 0.02), (0.79, 0.03, 0.01)]),
        equal_to(True),
        "consistent corrections did not resume mapping",
    )
