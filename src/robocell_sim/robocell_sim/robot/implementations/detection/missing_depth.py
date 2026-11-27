"""The failure to recover metric depth for a visible marker."""

from typing import final, override


@final
class MissingDepthError(ValueError):
    """A visible marker has no usable metric depth."""

    def __init__(self, message: str) -> None:
        """Store the reason the depth samples were rejected."""
        self._message = message

    @override
    def __str__(self) -> str:
        return f"Marker {self._message}"
