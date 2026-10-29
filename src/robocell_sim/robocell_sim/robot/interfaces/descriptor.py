"""The start of a standard-SCAN response."""

from typing import Protocol


class Descriptor(Protocol):
    """Descriptor that separates a validated stream header from its payload."""

    def payload(self) -> tuple[bytes, bool]:
        """Return available measurements and whether the header is pending."""
        ...
