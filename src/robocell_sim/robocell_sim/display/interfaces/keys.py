"""Keyboard events and manual drive controls."""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    import tkinter as tk


class Keys(Protocol):
    """Control the held keys, speeds and stop command."""

    def press(self, event: tk.Event[tk.Misc]) -> str:
        """Collapse X11 repeat pairs and accept fresh key presses."""
        ...

    def release(self, event: tk.Event[tk.Misc]) -> str:
        """Delay release until queued X11 repeat presses have been handled."""
        ...

    def forget(self, key: str) -> None:
        """Release a key once it is known to be a genuine release."""
        ...

    def stop(self) -> None:
        """Cancel movement until the keys are released and pressed again."""
        ...

    def adjust(self, key: str) -> None:
        """Treat a speed button as one complete press and release."""
        ...
