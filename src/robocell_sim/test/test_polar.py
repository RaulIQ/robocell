"""Tests for the polar chart window of the lidar scan."""

import multiprocessing
import secrets

from hamcrest import assert_that, equal_to
from robocell_sim.implementations.polar import Polar


def test_polar_queues_the_scan() -> None:
    """Marking a scan sends its ranges to the chart process."""
    queue = multiprocessing.get_context("fork").Queue(maxsize=2)
    scan = tuple(secrets.SystemRandom().uniform(0.12, 3.5) for _ in range(360))
    Polar(queue, 3.5).mark(scan)
    assert_that(
        queue.get(timeout=5),
        equal_to(scan),
        "chart did not receive the scan",
    )


def test_polar_drops_a_scan_when_the_chart_lags() -> None:
    """A full queue drops the new scan instead of holding the simulation."""
    queue = multiprocessing.get_context("fork").Queue(maxsize=1)
    first = tuple(secrets.SystemRandom().uniform(0.12, 3.5) for _ in range(360))
    Polar(queue, 3.5).mark(first)
    Polar(queue, 3.5).mark(
        tuple(secrets.SystemRandom().uniform(0.12, 3.5) for _ in range(360))
    )
    assert_that(
        queue.get(timeout=5),
        equal_to(first),
        "a full queue did not keep the earlier scan",
    )
