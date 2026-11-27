"""Gyro bias calibration from stamped IMU and wheel feedback."""

from dataclasses import dataclass
from logging import Logger
from typing import final, override

from rclpy.time import Time
from std_msgs.msg import String  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.implementations.imu.averaged import Averaged
from robocell_sim.robot.implementations.imu.zeroed import Zeroed
from robocell_sim.robot.interfaces.clock import Clock
from robocell_sim.robot.interfaces.zero import Zero
from robocell_sim.ros.implementations.imu.corrected import Corrected
from robocell_sim.ros.implementations.imu.rested import Rested
from robocell_sim.ros.interfaces.calibration import Calibration, Imu, JointState
from robocell_sim.ros.interfaces.topic import Topic

__all__ = ["Calibrated", "String", "zeroed"]

Limits = tuple[float, int, float]
State = tuple[Zero, str, int, bool]


def zeroed(limits: Limits) -> Zero:
    """Create the initial state of a stationary calibration interval."""
    return Zeroed(limits, (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3)))


def learned(zero: Zero, message: Imu, rested: Rested, moment: int) -> Zero:
    """Offer the angular velocity to the active bias estimator."""
    angular = message.angular_velocity
    return zero.read(
        (angular.x, angular.y, angular.z), moment * 1e-9, still=rested.still()
    )


def announced(zero: Zero, status: Topic, log: Logger) -> None:
    """Notify listeners that the stationary calibration changed state."""
    ready = zero.ready()
    status.publish(String(data="ready" if ready else "calibrating"))
    if ready:
        log.info("Gyro offsets are %.6f, %.6f and %.6f rad/s", *zero.bias())
    else:
        log.info("Gyro calibration requires a new stationary interval")


@final
@dataclass(frozen=True)
class Calibrated(Calibration):
    """Learn the gyro offset on stationary frames and publish corrected IMU."""

    _clock: Clock
    _configuration: tuple[Limits, tuple[float, float], tuple[str, str]]
    _output: tuple[Topic, Topic, Logger]
    _state: list[State]

    @override
    def read(self, message: Imu, feedback: JointState) -> None:
        zero, frame, stamp, ready = self._state[0]
        moment = Time.from_msg(message.header.stamp).nanoseconds
        if moment != stamp:
            if self.usable(message, moment):
                self.correct(message, feedback, moment)
            elif not zero.ready():
                self._state[0] = (
                    zeroed(self._configuration[0]),
                    frame,
                    stamp,
                    ready,
                )

    @override
    def usable(self, message: Imu, moment: int) -> bool:
        unavailable = message.angular_velocity_covariance[0] < 0
        return (
            abs(self._clock.now() - moment) * 1e-9 <= self._configuration[1][1]
            and not unavailable
        )

    @override
    def correct(self, message: Imu, feedback: JointState, moment: int) -> None:
        zero, frame, _, ready = self._state[0]
        if frame != message.header.frame_id:
            zero = zeroed(self._configuration[0])
        fitted = self.fit(zero, message, feedback, moment)
        if fitted.ready() != ready:
            announced(fitted, self._output[1], self._output[2])
        self._state[0] = fitted, message.header.frame_id, moment, fitted.ready()
        if fitted.ready():
            Corrected(message, fitted, self._output[0]).publish()

    @override
    def fit(
        self, zero: Zero, message: Imu, feedback: JointState, moment: int
    ) -> Zero:
        limits, thresholds, names = self._configuration
        try:
            return learned(
                zero,
                message,
                Rested(feedback, message, thresholds, names),
                moment,
            )
        except ValueError:
            self._output[2].warning(
                "Gyro calibration rejected a nonfinite IMU measurement"
            )
            return zeroed(limits)
