"""Guard updates preserve the age of the original application input."""

import math

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.command.arrows import Arrows
from robocell_sim.robot.implementations.command.filtered import Filtered
from robocell_sim.robot.implementations.drive.stepped import Stepped


@pytest.mark.parametrize("stamp", [-1, 499999999, 1000000001])
def test_filtered_inhibits_motion_without_a_fresh_scan(stamp: int) -> None:
    """Missing, expired or future scan data cannot permit motion."""
    assert_that(
        Filtered(
            Arrows(Stepped(0.047, 7, 3), Stepped(0.137, 9, -2)),
            13.71,
            ((math.inf,) * 360, stamp),
            (0.31, 0.21, 500000000),
        ).reading(1000000000, protected=True),
        equal_to((0.0, 0.0, 13.71)),
        "unavailable scan data did not inhibit both body velocities",
    )


def test_filtered_rechecks_a_new_obstacle_without_refreshing_the_command() -> (
    None
):
    """A scan update immediately overrides motion and retains input age."""
    control = Filtered(
        Arrows(Stepped(0.047, 7, 0), Stepped(0.137, 9, 0)),
        -1.0,
        ((), -1),
        (0.31, 0.21, 500000000),
    ).steer((0.173, -0.291), 13.71)
    control = control.scan((math.inf,) * 360, 1000000000)
    assert_that(
        control.scan((0.237, *((math.inf,) * 359)), 1200000000).reading(
            1210000000, protected=True
        ),
        equal_to((0.0, -0.291, 13.71)),
        "new obstacle did not stop translation without renewing input age",
    )


def test_filtered_ignores_an_older_command_from_another_source() -> None:
    """A delayed keyboard sample cannot supersede a newer ROS command."""
    assert_that(
        Filtered(
            Arrows(Stepped(0.047, 7, 0), Stepped(0.137, 9, 0)),
            -1.0,
            ((), -1),
            (0.31, 0.21, 500000000),
        )
        .steer((0.173, -0.291), 13.71)
        .steer((-0.071, 0.137), 13.69)
        .reading(-1, protected=False),
        equal_to((0.173, -0.291, 13.71)),
        "older source input did not leave the newer command intact",
    )
