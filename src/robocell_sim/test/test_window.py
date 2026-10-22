"""Tests for the window that shows the robot route."""

import multiprocessing
import secrets

from hamcrest import assert_that, equal_to
from robocell_sim.implementations.window import Window


def test_window_queues_both_positions() -> None:
    """Marking a point sends the encoder and the true position together."""
    queue = multiprocessing.get_context("fork").Queue()
    estimate = (
        secrets.SystemRandom().uniform(-5, 5),
        secrets.SystemRandom().uniform(-5, 5),
    )
    truth = (
        secrets.SystemRandom().uniform(-5, 5),
        secrets.SystemRandom().uniform(-5, 5),
    )
    Window(queue).mark(estimate, truth)
    assert_that(
        queue.get(timeout=5),
        equal_to((estimate, truth)),
        "window did not queue the encoder and true positions together",
    )
