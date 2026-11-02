"""Lifetime of one local CAN drive."""

from contextlib import AbstractContextManager
from typing import Protocol

from robocell_sim.hardware.interfaces.drive import Drive


class Attachment(Protocol):
    """Attachment that boots a drive and inhibits it on every exit path."""

    def connection(self) -> AbstractContextManager[Drive]:
        """Open a drive without taking ownership of its CAN bus."""
        ...
