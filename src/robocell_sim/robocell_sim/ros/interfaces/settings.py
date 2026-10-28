"""Settings of a ROS 2 node."""

from typing import Protocol


class Settings(Protocol):
    """Settings that a launch may override for one node.

    Each setting falls back to its default when the launch leaves it
    alone, so a node starts with sane values and no launch file at all.
    """

    def number(self, name: str, default: float) -> float:
        """Real number the launch set under the name, or the default."""
        ...

    def whole(self, name: str, default: int) -> int:
        """Whole number the launch set under the name, or the default."""
        ...

    def text(self, name: str, default: str) -> str:
        """Text the launch set under the name, or the default."""
        ...
