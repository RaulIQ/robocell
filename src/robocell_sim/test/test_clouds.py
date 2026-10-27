"""Tests for the chart of the two maps that grow from lidar scans."""

import multiprocessing
import secrets

import numpy as np
from hamcrest import assert_that, equal_to
from matplotlib.figure import Figure
from robocell_sim.display.implementations.maps.clouds import Clouds


def test_clouds_pile_up_the_true_map() -> None:
    """Drawing twice keeps every point by true pose from both batches."""
    queue = multiprocessing.get_context("fork").Queue()
    batches = [
        (
            [
                (
                    secrets.SystemRandom().uniform(-3, 3),
                    secrets.SystemRandom().uniform(-3, 3),
                )
                for _ in range(secrets.SystemRandom().randint(1, 9))
            ],
            [(0.0, 0.0)],
        )
        for _ in range(2)
    ]
    figure = Figure()
    lines = (
        figure.add_subplot(1, 2, 1).plot([], [])[0],
        figure.add_subplot(1, 2, 2).plot([], [])[0],
    )
    queue.put(batches[0])
    Clouds(queue, lines).draw()
    queue.put(batches[1])
    Clouds(queue, lines).draw()
    assert_that(
        np.asarray(lines[0].get_xydata()).tolist(),
        equal_to([list(point) for batch in batches for point in batch[0]]),
        "true map did not keep the points of every batch",
    )


def test_clouds_draw_the_reckoned_map_apart() -> None:
    """Points by reckoned pose go to the second chart only."""
    queue = multiprocessing.get_context("fork").Queue()
    estimate = [
        (
            secrets.SystemRandom().uniform(-3, 3),
            secrets.SystemRandom().uniform(-3, 3),
        )
        for _ in range(secrets.SystemRandom().randint(1, 9))
    ]
    figure = Figure()
    lines = (
        figure.add_subplot(1, 2, 1).plot([], [])[0],
        figure.add_subplot(1, 2, 2).plot([], [])[0],
    )
    queue.put(([(0.0, 0.0)], estimate))
    Clouds(queue, lines).draw()
    assert_that(
        np.asarray(lines[1].get_xydata()).tolist(),
        equal_to([list(point) for point in estimate]),
        "reckoned map did not land on the second chart",
    )
