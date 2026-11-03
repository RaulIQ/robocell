"""Tests for acquisition intervals shared across UART reads."""

import secrets

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.lidar.chronological import (
    Chronological,
)


def test_chronological_anchors_the_first_turn_to_its_estimate() -> None:
    """The first complete turn establishes the acquisition timeline."""
    stamp = secrets.randbelow(1_000_000) + 5_000_000_000
    assert_that(
        Chronological(-1, 0).read(stamp, 183_737_000, stamp + 190_000_000)[1],
        equal_to((stamp,)),
        "first turn did not anchor the acquisition timeline",
    )


def test_chronological_preserves_the_sample_period_despite_arrival_jitter() -> (
    None
):
    """A later read cannot move an established acquisition boundary."""
    stamp = secrets.randbelow(1_000_000) + 5_000_000_000
    timeline, _ = Chronological(-1, 0).read(
        stamp, 181_832_000, stamp + 181_832_000
    )
    assert_that(
        timeline.read(stamp - 19_173, 181_356_000, stamp + 400_000_000)[1],
        equal_to((stamp + 181_832_000,)),
        "arrival jitter did not preserve the established sample timeline",
    )


def test_chronological_discards_a_turn_that_extends_past_ros_time() -> None:
    """Queued bytes cannot imply acquisitions after the frozen clock."""
    timeline, _ = Chronological(-1, 0).read(
        9_818_168_000, 181_832_000, 10_000_000_000
    )
    assert_that(
        timeline.read(9_810_000_000, 181_356_000, 10_000_000_000)[1],
        equal_to(()),
        "turn beyond the paused clock did not get discarded",
    )


def test_chronological_reanchors_after_an_unplaceable_turn() -> None:
    """Resumed acquisitions use a fresh estimate after contradictory data."""
    timeline, _ = Chronological(-1, 0).read(
        9_818_168_000, 181_832_000, 10_000_000_000
    )
    timeline, _ = timeline.read(9_810_000_000, 181_356_000, 10_000_000_000)
    assert_that(
        timeline.read(10_250_000_000, 181_832_000, 10_500_000_000)[1],
        equal_to((10_250_000_000,)),
        "resumed turn did not establish a fresh acquisition estimate",
    )


def test_chronological_preserves_published_history_after_reset() -> None:
    """Resynchronization cannot reuse an already published interval."""
    timeline, _ = Chronological(-1, 0).read(
        9_818_168_000, 181_832_000, 10_000_000_000
    )
    assert_that(
        timeline.reset().read(9_800_000_000, 181_356_000, 10_000_000_000)[1],
        equal_to(()),
        "resynchronization did not preserve the published interval boundary",
    )
