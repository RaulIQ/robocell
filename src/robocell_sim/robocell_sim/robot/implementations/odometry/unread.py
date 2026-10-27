"""Odometry before the first reading of the wheels."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.odometry.gyroscopic import Gyroscopic
from robocell_sim.robot.interfaces.kinematics import Kinematics
from robocell_sim.robot.interfaces.odometry import Odometry


@final
@dataclass(frozen=True)
class Unread(Odometry):
    """Odometry that has read neither the encoders nor the gyro yet.

    The kinematics and the resolution pass on to the odometry that follows,
    and the pose holds x and y in meters and the heading in radians where
    the robot starts. The first reading only sets the counts and the angle
    to count from, so a driver that starts with the wheels already turned
    does not leap across the room.
    """

    _kinematics: Kinematics
    _resolution: int
    _pose: tuple[float, float, float]

    @override
    def read(self, left: int, right: int, angle: float) -> Odometry:
        return Gyroscopic(
            self._kinematics, self._resolution, self._pose, (left, right, angle)
        )

    @override
    def position(self) -> tuple[float, float]:
        return (self._pose[0], self._pose[1])

    @override
    def heading(self) -> float:
        return self._pose[2]
