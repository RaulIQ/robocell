"""Bounded host and physical-drive integration experiment."""

from typing import Protocol

from robocell_sim.robot.interfaces.pilot import Joints


class Flight(Protocol):
    """Physical CAN integration with independent emulator endpoints."""

    def enable(self) -> str:
        """Wait a bounded time for the verified startup sequence."""
        ...

    def advance(
        self, duration: float, command: tuple[tuple[float, float], ...]
    ) -> tuple[str, tuple[Joints, ...]]:
        """Advance physics and both sides of CAN for a wall-time interval."""
        ...

    def step(
        self, command: tuple[tuple[float, float], ...]
    ) -> tuple[str, tuple[Joints, ...]]:
        """Service both endpoints around one physical step."""
        ...
