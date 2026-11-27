"""The simulator time received independently of drive feedback."""

from dataclasses import dataclass
from queue import SimpleQueue
from typing import final

from rosgraph_msgs.msg import Clock  # type: ignore[import-untyped]  # ROS generates message modules without types

__all__ = ["Clock", "Received"]


@final
@dataclass(frozen=True)
class Received:
    """Translate each simulation clock message to an integer timestamp."""

    _inbox: SimpleQueue[int]

    def __call__(self, message: Clock) -> None:
        """Queue the timestamp without relying on any wheel acquisition."""
        self._inbox.put(
            message.clock.sec * 1_000_000_000 + message.clock.nanosec
        )
