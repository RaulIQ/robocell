"""Acquired points deskewed through the odometry TF history."""

from dataclasses import dataclass
from typing import final, override

from rclpy.time import Time
from tf2_ros import Buffer  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.ros.interfaces.coordinates import Coordinates


@final
@dataclass(frozen=True)
class Transformed(Coordinates):
    """Coordinates at a common sensor pose using interpolated TF samples."""

    _buffer: Buffer
    _frame: str
    _fixed: str

    @override
    def point(
        self, point: tuple[float, float], stamp: int, reference: int
    ) -> tuple[float, float]:
        transform = self._buffer.lookup_transform_full(
            self._frame,
            Time(nanoseconds=reference),
            self._frame,
            Time(nanoseconds=stamp),
            self._fixed,
        ).transform
        rotation = transform.rotation
        return (
            (1 - 2 * (rotation.y**2 + rotation.z**2)) * point[0]
            + 2 * (rotation.x * rotation.y - rotation.z * rotation.w) * point[1]
            + transform.translation.x,
            2 * (rotation.x * rotation.y + rotation.z * rotation.w) * point[0]
            + (1 - 2 * (rotation.x**2 + rotation.z**2)) * point[1]
            + transform.translation.y,
        )
