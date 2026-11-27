"""Navigation callbacks and the control panel status contract."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from rclpy.task import Future


class Following(Protocol):
    """Submit and track a Nav2 action for the selected target."""

    def transition(self) -> None:
        """Apply the operator's latest request."""
        ...

    def start(self) -> None:
        """Submit a fresh target goal when the server is ready."""
        ...

    def accepted(self, future: Future) -> None:
        """Handle the goal acknowledgement."""
        ...

    def cancelled(self, future: Future) -> None:
        """Handle the goal cancellation response."""
        ...

    def finished(self, future: Future) -> None:
        """Clear the completed goal and update follow state."""
        ...
