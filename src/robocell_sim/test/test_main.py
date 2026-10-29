"""Tests for optional windows at simulator startup."""

from pathlib import Path

import pytest
from hamcrest import assert_that, equal_to

from test.booted import Booted


@pytest.mark.parametrize(
    ("flags", "count"),
    [
        ((), 0),
        (("--route",), 1),
        (("--camera",), 1),
        (("--radar",), 1),
        (("--survey",), 1),
        (("--route", "--camera", "--radar", "--survey"), 4),
        (
            (
                "--route=false",
                "--camera=false",
                "--radar=false",
                "--survey=false",
            ),
            0,
        ),
        (("--route=true", "--camera=false", "--survey=true"), 2),
    ],
)
def test_main_starts_and_feeds_only_requested_windows(
    flags: tuple[str, ...],
    count: int,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    """Disabled windows allocate no queues; enabled ones start before ROS."""
    assert_that(
        Booted(flags, tmp_path, monkeypatch).events(),
        equal_to((count, [False] * count, [1] * count)),
        "simulator did not start and feed only requested windows before ROS",
    )
