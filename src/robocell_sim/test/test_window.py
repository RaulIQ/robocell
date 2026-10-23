"""Tests for the window that shows the robot route."""

import multiprocessing
import secrets

from hamcrest import assert_that, equal_to
from robocell_sim.display.implementations.window import Window


def test_window_queues_both_positions_and_the_slip() -> None:
    """Marking a point sends both positions and the slip together."""
    queue = multiprocessing.get_context("fork").Queue()
    estimate = (
        secrets.SystemRandom().uniform(-5, 5),
        secrets.SystemRandom().uniform(-5, 5),
    )
    truth = (
        secrets.SystemRandom().uniform(-5, 5),
        secrets.SystemRandom().uniform(-5, 5),
    )
    slipping = secrets.choice((True, False))
    Window(queue).mark(estimate, truth, slipping=slipping)
    assert_that(
        queue.get(timeout=5),
        equal_to((estimate, truth, slipping)),
        "window did not queue both positions and the slip together",
    )
