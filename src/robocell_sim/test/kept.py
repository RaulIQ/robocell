"""Topic that keeps its messages in memory."""

from dataclasses import dataclass
from typing import Any, final, override

from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Kept(Topic):
    """Topic that appends every message to a list instead of sending it.

    The list receives the messages in the order they were published, so a
    test reads what a ROS 2 subscriber would have received.
    """

    _messages: list[Any]

    @override
    def publish(self, message: object, /) -> None:
        self._messages.append(message)
