"""Intrinsics of a camera."""

from typing import Protocol


class Intrinsics(Protocol):
    """Intrinsics that project points in the camera frame onto pixels."""

    def matrix(
        self,
    ) -> tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]:
        """Camera matrix with focal lengths and principal point in pixels."""
