"""CAN device serviced on its physics owner's thread."""

from typing import Protocol


class Drive(Protocol):
    """Device that bounds CAN service and protects its actuator."""

    def advance(self, moment: float) -> None:
        """Service pending frames and protection using monotonic seconds."""
        ...

    def close(self) -> None:
        """Inhibit output and release protocol timers."""
        ...
