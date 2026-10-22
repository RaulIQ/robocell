"""Tests for the chart of the encoder and true tracks."""

import multiprocessing
import secrets

import numpy as np
from hamcrest import assert_that, close_to, equal_to
from matplotlib.figure import Figure
from robocell_sim.implementations.tracks import Tracks


def test_tracks_draw_the_true_route() -> None:
    """Drawing appends every queued true position to the true track."""
    queue = multiprocessing.get_context("fork").Queue()
    route = [
        (
            (secrets.SystemRandom().uniform(-5, 5), 0.0),
            (
                secrets.SystemRandom().uniform(-5, 5),
                secrets.SystemRandom().uniform(-5, 5),
            ),
        )
        for _ in range(secrets.SystemRandom().randint(2, 9))
    ]
    axes = Figure().subplots()
    lines = (axes.plot([], [])[0], axes.plot([], [])[0])
    for point in route:
        queue.put(point)
    Tracks(queue, axes, lines).draw()
    assert_that(
        np.asarray(lines[1].get_xydata()).tolist(),
        equal_to([list(truth) for _, truth in route]),
        "true track did not receive every queued position",
    )


def test_tracks_title_the_latest_error() -> None:
    """The title tells how far the encoders are off in millimeters."""
    queue = multiprocessing.get_context("fork").Queue()
    gap = secrets.SystemRandom().uniform(0.001, 0.5)
    axes = Figure().subplots()
    queue.put(((gap, 0.0), (0.0, 0.0)))
    Tracks(queue, axes, (axes.plot([], [])[0], axes.plot([], [])[0])).draw()
    assert_that(
        float(axes.get_title().split()[2]),
        close_to(gap * 1000, 0.5),
        "title did not tell the encoder error in millimeters",
    )
