"""Feedback from target following to the operator's interface."""

from typing import Protocol


class FollowView(Protocol):
    """Display target status and update the follow control."""

    def status(self, text: str) -> None:
        """Display the current target or action state."""
        ...

    def button(self, text: str) -> None:
        """Update the text on the follow control."""
        ...

    def enabled(self, *, value: bool) -> None:
        """Enable the follow control when it can accept another request."""
        ...
