"""Kinematics of a wheeled robot."""

from typing import Protocol


class Kinematics(Protocol):
    """Link between the body velocity of a robot and the speeds of its wheels.

    The linear speed points forward in m/s, the angular speed turns
    counterclockwise around the vertical axis in rad/s, and the wheel
    speeds are in rad/s.
    """

    def wheels(self, linear: float, angular: float) -> tuple[float, float]:
        """Left and right wheel speeds that produce a body velocity."""
        ...

    def velocity(self, left: float, right: float) -> tuple[float, float]:
        """Linear and angular body velocity that the wheel speeds produce."""
        ...
