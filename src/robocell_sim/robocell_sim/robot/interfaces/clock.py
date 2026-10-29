"""Time observed by a robot process."""

from typing import Protocol


class Clock(Protocol):
    """Clock that supplies the current ROS or hardware time in nanoseconds."""

    def now(self) -> int:
        """Read the current time."""
        ...
