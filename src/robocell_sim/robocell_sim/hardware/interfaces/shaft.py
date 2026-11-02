"""Physical shaft controlled by a drive emulator."""

from typing import Protocol


class Shaft(Protocol):
    """Shaft whose commanded speed and measured motion remain distinct."""

    def spin(self, velocity: float) -> None:
        """Apply a velocity target in radians per second."""
        ...

    def measurement(self) -> tuple[float, float, float]:
        """Return angle, actual velocity and acquisition time in SI units."""
        ...

    def paused(self) -> bool:
        """Report whether physics has been explicitly paused."""
        ...
