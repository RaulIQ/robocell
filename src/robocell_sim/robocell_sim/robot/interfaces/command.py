"""Velocity command of an operator."""

from typing import Protocol


class Command(Protocol):
    """Body velocity that an operator commands by key presses or directly."""

    def press(self, key: int) -> "Command":
        """Command after one more press of a GLFW key code."""
        ...

    def steer(self, linear: float, angular: float) -> "Command":
        """Command after the operator sets the velocity in m/s and rad/s."""
        ...

    def velocity(self) -> tuple[float, float]:
        """Forward speed in m/s and counterclockwise yaw rate in rad/s."""
        ...
