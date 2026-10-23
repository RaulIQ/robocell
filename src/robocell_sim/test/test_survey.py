"""Tests for the window that draws the maps by true and reckoned pose."""

import multiprocessing
import secrets

from hamcrest import assert_that, equal_to
from robocell_sim.display.implementations.survey import Survey


def test_survey_queues_both_maps() -> None:
    """Marking sends the points by true and by reckoned pose together."""
    queue = multiprocessing.get_context("fork").Queue()
    truth = [
        (
            secrets.SystemRandom().uniform(-3, 3),
            secrets.SystemRandom().uniform(-3, 3),
        )
        for _ in range(secrets.SystemRandom().randint(1, 9))
    ]
    estimate = [
        (
            secrets.SystemRandom().uniform(-3, 3),
            secrets.SystemRandom().uniform(-3, 3),
        )
        for _ in range(secrets.SystemRandom().randint(1, 9))
    ]
    Survey(queue).mark(truth, estimate)
    assert_that(
        queue.get(timeout=5),
        equal_to((truth, estimate)),
        "window did not receive both maps together",
    )
