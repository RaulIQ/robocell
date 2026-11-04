"""A real TF history with an explicitly chosen planar displacement."""

import math
from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from rclpy.time import Time
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types
from tf2_ros import Buffer  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.history import History


@final
@dataclass(frozen=True)
class Trajectory(History):
    """Two measured poses enclosing a scan's acquisition interval."""

    _start: int
    _finish: int
    _translation: tuple[float, float]
    _rotation: float

    @override
    def buffer(self) -> Buffer:
        """Build a real TF buffer with interpolation between the poses."""
        buffer = Buffer()
        for stamp, position, angle in (
            (self._start, (0.0, 0.0), 0.0),
            (self._finish, self._translation, self._rotation),
        ):
            buffer.set_transform(
                TransformStamped(
                    header=Header(
                        stamp=Time(nanoseconds=stamp).to_msg(),
                        frame_id="odom",
                    ),
                    child_frame_id="base_scan",
                    transform=Transform(
                        translation=Vector3(x=position[0], y=position[1]),
                        rotation=Quaternion(
                            z=math.sin(angle / 2), w=math.cos(angle / 2)
                        ),
                    ),
                ),
                "test",
            )
        return buffer
