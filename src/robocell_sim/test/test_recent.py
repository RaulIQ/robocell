"""Tests for when an observation can still be used."""

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.detection.recent import Recent

from test.instant import Instant


def test_recent_rejects_delayed_and_future_frames() -> None:
    """Only timestamps within the configured age window are usable."""
    recent = Recent(Instant(12.5), 430_000_000)
    assert_that(
        (
            recent(12_070_000_000),
            recent(12_069_999_999),
            recent(12_500_000_001),
        ),
        equal_to((True, False, False)),
        "the age window did not reject delayed and future observations",
    )
