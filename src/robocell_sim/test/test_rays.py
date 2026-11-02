"""LaserScan samples enter the existing one-degree collision geometry."""

import math

from hamcrest import assert_that, close_to
from robocell_sim.ros.implementations.command.rays import Rays
from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types


def test_rays_preserves_front_and_rear_distances() -> None:
    """Clockwise scans and duplicate bearings retain the nearest valid echo."""
    assert_that(
        math.dist(
            tuple(
                Rays(
                    LaserScan(
                        angle_min=math.pi,
                        angle_increment=-math.pi,
                        range_min=0.15,
                        range_max=12.0,
                        ranges=[0.237, 0.291, 0.173],
                    )
                ).ranges()[index]
                for index in (0, 180)
            ),
            (0.291, 0.173),
        ),
        close_to(0.0, 1e-6),
        "scan conversion did not preserve the nearest front and rear echoes",
    )
