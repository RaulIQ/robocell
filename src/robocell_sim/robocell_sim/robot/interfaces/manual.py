"""Held manual input independent of its graphical presentation."""

from typing import Protocol


class Manual(Protocol):
    """Immutable direction state and selected movement speeds."""

    def press(self, key: str) -> "Manual":
        """Accept a newly pressed direction, adjustment or stop key."""
        ...

    def release(self, key: str) -> "Manual":
        """Forget a released key, including a cancelled held direction."""
        ...

    def velocity(self) -> tuple[float, float]:
        """Return the body velocity requested by held directions."""
        ...

    def speeds(self) -> tuple[float, float]:
        """Return the selected positive linear and angular settings."""
        ...
