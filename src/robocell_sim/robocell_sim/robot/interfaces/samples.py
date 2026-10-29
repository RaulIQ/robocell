"""Boundaries in an incoming measurement stream."""

from typing import Protocol


class Samples(Protocol):
    """Samples that retain incomplete bytes and identify alignment loss."""

    def split(
        self,
    ) -> tuple[tuple[tuple[bool, bool, float, float, int], ...], bytes]:
        """Decode measurements and loss markers, leaving the partial tail."""
        ...
