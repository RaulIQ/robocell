"""Observations from a simulator startup."""

import multiprocessing.context
import multiprocessing.queues
from typing import Protocol


class Boot(Protocol):
    """Startup whose observations record window resources and data."""

    def events(self) -> tuple[int, list[bool], list[int]]:
        """Return queue count, ROS state at each fork and frame counts."""
        ...

    def queues(
        self,
        context: multiprocessing.context.BaseContext,
        queues: list[multiprocessing.queues.Queue[object]],
    ) -> None:
        """Record the real queues allocated for windows."""
        ...

    def processes(self, starts: list[bool]) -> None:
        """Observe forks without opening GUI processes."""
        ...

    def viewer(self) -> None:
        """Supply a viewer that stops after one frame."""
        ...

    def environment(self) -> None:
        """Make package resources available without a colcon build."""
        ...
