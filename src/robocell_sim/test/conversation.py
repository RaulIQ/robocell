"""A recorded exchange with a device."""

from typing import Protocol


class Conversation(Protocol):
    """Conversation that records the reply to each host action."""

    def responses(self) -> tuple[bytes, ...]:
        """Run the bounded exchange and return each response."""
        ...
