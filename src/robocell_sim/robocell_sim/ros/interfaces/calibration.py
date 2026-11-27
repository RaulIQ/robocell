"""The raw ROS measurements needed to calibrate a gyroscope."""

from typing import Protocol

from sensor_msgs.msg import Imu, JointState  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.interfaces.zero import Zero

__all__ = ["Calibration", "Imu", "JointState"]


class Calibration(Protocol):
    """Learn a stationary gyro bias and publish corrected readings."""

    def read(self, message: Imu, feedback: JointState) -> None:
        """Process one IMU reading and the latest wheel feedback."""
        ...

    def usable(self, message: Imu, moment: int) -> bool:
        """Check the measurement stamp and angular covariance."""
        ...

    def correct(self, message: Imu, feedback: JointState, moment: int) -> None:
        """Advance the calibration and publish the corrected measurement."""
        ...

    def fit(
        self, zero: Zero, message: Imu, feedback: JointState, moment: int
    ) -> Zero:
        """Fit the stationary bias sample or restart the interval."""
        ...
