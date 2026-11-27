"""Target localization at the acquisition timestamp of a camera frame."""

from dataclasses import dataclass
from typing import final

from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped  # type: ignore[import-untyped]  # Jazzy ships this module without typing information

from robocell_sim.ros.interfaces.sighting import Sighting
from robocell_sim.ros.messages import (
    Buffer,
    ExtrapolationException,
    PoseStamped,
    TransformException,
)


def awaiting(buffer: Buffer, frames: tuple[str, str], stamp: Time) -> bool:
    """Check whether TF has yet to reach the acquisition time."""
    try:
        latest = buffer.lookup_transform(*frames, Time())
    except TransformException:
        return False
    return Time.from_msg(latest.header.stamp) < stamp


@final
@dataclass(frozen=True)
class Located:
    """Transform an observed pose into the navigation frame and report it."""

    _buffer: Buffer
    _frame: str
    _output: Sighting

    def __call__(self, goal: PoseStamped) -> None:
        """Publish only if TF has a transform at the observation time."""
        frames, stamp = (
            (self._frame, goal.header.frame_id),
            Time.from_msg(goal.header.stamp),
        )
        try:
            self._output.publish(
                do_transform_pose_stamped(
                    goal, self._buffer.lookup_transform(*frames, stamp)
                )
            )
        except ExtrapolationException as error:
            extrapolated(
                self._output,
                error,
                pending=awaiting(self._buffer, frames, stamp),
            )
        except TransformException as error:
            self._output.unavailable(error)


def extrapolated(
    output: Sighting, error: ExtrapolationException, *, pending: bool
) -> None:
    """Report extrapolation for acquisitions within TF history."""
    if not pending:
        output.unavailable(error)
