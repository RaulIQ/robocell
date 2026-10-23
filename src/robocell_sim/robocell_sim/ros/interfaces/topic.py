"""Topic of ROS 2."""

from typing import Protocol


class Topic(Protocol):
    """Topic that carries messages of one type to whoever subscribes."""

    def publish(self, message: object, /) -> None:
        """Send one message to every subscriber of the topic."""
        ...
