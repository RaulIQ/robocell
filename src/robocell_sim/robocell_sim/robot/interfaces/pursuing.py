"""The operator and target events that govern a navigation goal."""

from typing import Literal, Protocol

Stage = Literal["idle", "pending", "active"]
Decision = Literal["start", "cancel", "wait"]


class Pursuing(Protocol):
    """Track the operator request, target visibility and goal progress."""

    def toggle(self) -> "Pursuing":
        """Reverse the operator's follow request."""
        ...

    def sight(self, *, visible: bool) -> "Pursuing":
        """Record whether a usable target is visible."""
        ...

    def goal(self, stage: Stage) -> "Pursuing":
        """Record the goal's action state."""
        ...

    def decision(self) -> Decision:
        """Decide whether Nav2 should start, cancel or wait."""
        ...

    def requested(self) -> bool:
        """Tell the panel whether the operator has requested following."""
        ...
