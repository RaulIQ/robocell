"""Body commands retain their source time."""

from hamcrest import assert_that, equal_to
from robocell_sim.robot.implementations.command.arrows import Arrows
from robocell_sim.robot.implementations.command.filtered import Filtered
from robocell_sim.robot.implementations.drive.stepped import Stepped


def test_filtered_ignores_an_older_command_from_another_source() -> None:
    """A delayed keyboard sample cannot supersede a newer ROS command."""
    assert_that(
        Filtered(
            Arrows(Stepped(0.047, 7, 0), Stepped(0.137, 9, 0)),
            -1.0,
        )
        .steer((0.173, -0.291), 13.71)
        .steer((-0.071, 0.137), 13.69)
        .reading(),
        equal_to((0.173, -0.291, 13.71)),
        "older source input did not leave the newer command intact",
    )
