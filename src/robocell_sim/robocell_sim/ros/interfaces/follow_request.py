"""Operator requests and target observations used to decide when to follow."""

from typing import Protocol

from std_msgs.msg import String  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.messages import PoseStamped

__all__ = ["FollowRequest", "String"]


class FollowRequest(Protocol):
    """Accept changes to operator intent and target visibility."""

    def observe(self, message: PoseStamped) -> None:
        """Remember a fresh target and reconsider the current goal."""
        ...

    def status(self, message: String) -> None:
        """Track detector status and cancel following when necessary."""
        ...

    def lose(self, reason: str) -> None:
        """Discard the current target without undoing the follow request."""
        ...

    def toggle(self) -> None:
        """Toggle the operator's follow request."""
        ...

    def expire(self) -> None:
        """Discard a target if no fresh goal has arrived."""
        ...
