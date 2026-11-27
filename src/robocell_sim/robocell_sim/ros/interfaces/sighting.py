"""State and output of visual target tracking."""

from typing import Protocol

from robocell_sim.ros.messages import PoseStamped, TransformException


class Sighting(Protocol):
    """Publish a usable target or the reason it is unavailable."""

    def report(self, current: str) -> None:
        """Publish changes to target availability."""
        ...

    def unavailable(self, error: TransformException) -> None:
        """Report a failed frame transform."""
        ...

    def expire(self) -> None:
        """Expire the sighting when valid updates stop arriving."""
        ...

    def publish(self, target: PoseStamped) -> None:
        """Publish a transformed target and its diagnostic marker."""
        ...
