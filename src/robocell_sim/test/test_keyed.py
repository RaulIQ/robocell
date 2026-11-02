"""Keyboard publication preserves the real source-event timestamp."""

from typing import Any

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.command.arrows import Arrows
from robocell_sim.robot.implementations.drive.stepped import Stepped
from robocell_sim.ros.implementations.command.keyed import Keyed

from test.kept import Kept


def test_keyed_publishes_original_key_time_and_raw_body_intent() -> None:
    """The keyboard publisher cannot refresh a queued event's timestamp."""
    messages: list[Any] = []
    Keyed(
        Arrows(Stepped(0.047, 7, 3), Stepped(0.137, 9, -2)),
        1730012345987654321,
        Kept(messages),
    ).publish()
    assert_that(
        (
            messages[0].header.stamp.sec,
            messages[0].header.stamp.nanosec,
            messages[0].twist.linear.x,
            messages[0].twist.angular.z,
        ),
        equal_to((1730012345, 987654321, 3 * 0.047, -2 * 0.137)),
        "keyboard publication did not preserve source time and body intent",
    )
