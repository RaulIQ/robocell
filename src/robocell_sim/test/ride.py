"""Ride of the simulated robot."""

from typing import Protocol

import mujoco


class Ride(Protocol):
    """Run of the simulated robot from rest with fixed wheel commands."""

    def state(self) -> mujoco.MjData:
        """State of the simulation at the end of the ride."""

    def velocity(self) -> tuple[float, float]:
        """Forward speed in m/s and yaw rate in rad/s in the robot frame."""

    def wheels(self) -> tuple[float, float]:
        """Left and right wheel joint speeds in rad/s."""
