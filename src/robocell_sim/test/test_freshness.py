"""Tests for rejecting stale and future target observations."""

from hamcrest import assert_that, equal_to
from robocell_sim.freshness import fresh


def test_freshness_rejects_delayed_and_future_frames() -> None:
    """Only observations within the half-second age window remain usable."""
    assert_that(
        (fresh(12.0, 12.5), fresh(12.0, 12.500000001), fresh(12.0, 11.9)),
        equal_to((True, False, False)),
        "frame freshness did not reject delayed and future observations",
    )
