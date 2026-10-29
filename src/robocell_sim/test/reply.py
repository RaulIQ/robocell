"""The bytes returned to a host request."""

from typing import Protocol


class Reply(Protocol):
    """Reply that collects bytes from the device under a bounded deadline."""

    def receive(self) -> bytes:
        """Collect the reply until it becomes quiet or reaches the deadline."""
        ...
