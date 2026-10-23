"""Velocity command of an operator."""

from typing import Protocol


class Command(Protocol):
    """Body velocity that an operator commands one key press at a time."""

    def press(self, key: int) -> "Command":
        """Command after one more press of a GLFW key code."""

    def velocity(self) -> tuple[float, float]:
        """Forward speed in m/s and counterclockwise yaw rate in rad/s."""
