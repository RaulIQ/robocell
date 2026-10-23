"""Tests for the chart of the latest lidar sweep."""

import math
import multiprocessing
import secrets

import numpy as np
from hamcrest import assert_that, equal_to
from matplotlib.figure import Figure
from robocell_sim.implementations.sweep import Sweep


def test_sweep_draws_the_echoes_of_the_scan() -> None:
    """Drawing places every valid range at its angle around the robot."""
    queue = multiprocessing.get_context("fork").Queue()
    scan = tuple(
        secrets.SystemRandom().uniform(0.12, 3.5)
        if secrets.choice((True, False))
        else math.inf
        for _ in range(360)
    )
    line = Figure().add_subplot(projection="polar").plot([], [])[0]
    queue.put(scan)
    Sweep(queue, line).draw()
    assert_that(
        np.asarray(line.get_xydata()).tolist(),
        equal_to(
            [
                [math.radians(index), distance]
                for index, distance in enumerate(scan)
                if math.isfinite(distance)
            ]
        ),
        "sweep did not place every valid range at its angle",
    )


def test_sweep_keeps_only_the_latest_scan() -> None:
    """Drawing after several scans shows the last one alone."""
    queue = multiprocessing.get_context("fork").Queue()
    scans = [
        tuple(secrets.SystemRandom().uniform(0.12, 3.5) for _ in range(360))
        for _ in range(secrets.SystemRandom().randint(2, 5))
    ]
    line = Figure().add_subplot(projection="polar").plot([], [])[0]
    for scan in scans:
        queue.put(scan)
    Sweep(queue, line).draw()
    assert_that(
        np.asarray(line.get_ydata()).tolist(),
        equal_to(list(scans[-1])),
        "sweep did not show the latest scan alone",
    )


def test_sweep_keeps_the_last_scan_when_none_arrived() -> None:
    """Drawing with an empty queue leaves the chart as it was."""
    queue = multiprocessing.get_context("fork").Queue()
    scan = tuple(secrets.SystemRandom().uniform(0.12, 3.5) for _ in range(360))
    line = Figure().add_subplot(projection="polar").plot([], [])[0]
    queue.put(scan)
    Sweep(queue, line).draw()
    Sweep(queue, line).draw()
    assert_that(
        np.asarray(line.get_ydata()).tolist(),
        equal_to(list(scan)),
        "an empty queue did not leave the chart as it was",
    )
