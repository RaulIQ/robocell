"""Tests for the journal that warns about lost measurements in a log."""

import logging
import logging.handlers
import secrets

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.imu.logged import Logged


def test_logged_warns_about_lost_measurements() -> None:
    """A loss turns into one warning that names how many went missing."""
    count = secrets.SystemRandom().randint(1, 255)
    log = logging.getLogger(secrets.token_hex(8))
    buffer = logging.handlers.BufferingHandler(8)
    log.addHandler(buffer)
    Logged(log).lose(count)
    assert_that(
        [record.getMessage() for record in buffer.buffer],
        equal_to([f"IMU lost {count} measurements"]),
        "journal did not warn about the lost measurements",
    )


def test_logged_stays_quiet_without_a_loss() -> None:
    """A frame that lost nothing leaves the log alone."""
    log = logging.getLogger(secrets.token_hex(8))
    log.setLevel(logging.DEBUG)
    buffer = logging.handlers.BufferingHandler(8)
    log.addHandler(buffer)
    Logged(log).lose(0)
    assert_that(
        buffer.buffer,
        equal_to([]),
        "journal did not stay quiet without a loss",
    )
