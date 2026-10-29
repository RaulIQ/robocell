"""A lidar command session."""

from typing import Protocol


class Connection(Protocol):
    """Connection that negotiates the standard sampling stream."""

    def start(self) -> int:
        """Start standard scanning and return its sample period."""
        ...

    def stop(self) -> None:
        """Ask the device to stop producing measurements."""
        ...
