"""Report that publishes an IMU reading."""

from dataclasses import dataclass
from typing import final, override

from geometry_msgs.msg import Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from sensor_msgs.msg import Imu  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.reading import Reading
from robocell_sim.ros.interfaces.topic import Topic


@final
@dataclass(frozen=True)
class Inertial(Report):
    """Report that publishes one reading of the IMU as sensor_msgs/Imu.

    The reading gives the angular velocity and the linear acceleration, the
    header the stamp and the frame, the variances those of the gyro and of
    the accelerometer in that order, and the topic carries the message. The
    board sends no orientation, so the first element of its covariance is
    -1, which tells readers to ignore it.
    """

    _reading: Reading
    _header: Header
    _variances: tuple[float, float]
    _topic: Topic

    @override
    def publish(self) -> None:
        self._topic.publish(
            Imu(
                header=self._header,
                orientation_covariance=[-1.0] + [0.0] * 8,
                angular_velocity=Vector3(
                    x=self._reading.rate()[0],
                    y=self._reading.rate()[1],
                    z=self._reading.rate()[2],
                ),
                angular_velocity_covariance=[
                    self._variances[0] * (row == column)
                    for row in range(3)
                    for column in range(3)
                ],
                linear_acceleration=Vector3(
                    x=self._reading.acceleration()[0],
                    y=self._reading.acceleration()[1],
                    z=self._reading.acceleration()[2],
                ),
                linear_acceleration_covariance=[
                    self._variances[1] * (row == column)
                    for row in range(3)
                    for column in range(3)
                ],
            )
        )
