"""Keyboard input retains its source wall age across the ROS boundary."""

from geometry_msgs.msg import Twist, TwistStamped, Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from hamcrest import assert_that, equal_to
from rclpy.time import Time
from robocell_sim.ros.implementations.command.stamped import Stamped
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types


def test_stamped_preserves_source_age_after_transport_delay() -> None:
    """A delayed keyboard message cannot acquire a fresh command deadline."""
    assert_that(
        Stamped(
            TwistStamped(
                header=Header(
                    stamp=Time(nanoseconds=1730012345000000000).to_msg()
                ),
                twist=Twist(linear=Vector3(x=0.173), angular=Vector3(z=-0.291)),
            ),
            (13.71, 1730012345073000000),
        ).command(),
        equal_to((0.173, -0.291, 13.71 - 0.073)),
        "keyboard transport did not retain the source command age",
    )
