"""CAN acquisition data published as ROS joint feedback."""

from typing import Any

from hamcrest import assert_that, equal_to
from robocell_sim.ros.implementations.motor.reported import Reported

from test.kept import Kept


def test_motor_reported_preserves_acquisition_and_measured_wheel_values() -> (
    None
):
    """The ROS message carries the CAN acquisition time and wheel order."""
    messages: list[Any] = []
    Reported(
        (12345678901, (-0.713, 2.371), (-1.937, 0.719)),
        ("left_custom", "right_custom"),
        Kept(messages),
    ).publish()
    assert_that(
        (
            messages[0].header.stamp.sec,
            messages[0].header.stamp.nanosec,
            tuple(messages[0].name),
            tuple(messages[0].position),
            tuple(messages[0].velocity),
        ),
        equal_to(
            (
                12,
                345678901,
                ("left_custom", "right_custom"),
                (-0.713, 2.371),
                (-1.937, 0.719),
            )
        ),
        "joint message did not preserve CAN acquisition and wheel values",
    )
