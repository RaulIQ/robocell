"""Kinematics of a wheeled robot."""

from typing import Protocol


class Kinematics(Protocol):
    """Map from a body velocity of a robot to the speeds of its wheels."""

    def wheels(self, linear: float, angular: float) -> tuple[float, float]:
        """Left and right wheel speeds in rad/s for a body velocity.

        The linear speed points forward in m/s and the angular speed turns
        counterclockwise around the vertical axis in rad/s.
        """
