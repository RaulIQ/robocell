"""Tests for publishing the calibrated IMU stream."""

from typing import TYPE_CHECKING, Any

from geometry_msgs.msg import Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from hamcrest import assert_that, close_to, equal_to
from robocell_sim.robot.implementations.imu.averaged import Averaged
from robocell_sim.robot.implementations.imu.zeroed import Zeroed
from robocell_sim.ros.implementations.imu.corrected import Corrected
from sensor_msgs.msg import Imu  # type: ignore[import-untyped]  # ROS generates message modules without types
from std_msgs.msg import Header  # type: ignore[import-untyped]  # ROS generates message modules without types

from test.kept import Kept

if TYPE_CHECKING:
    from robocell_sim.robot.interfaces.zero import Zero


def test_corrected_preserves_a_real_turn_after_calibration() -> None:
    """Only the learned stationary offset is subtracted during a turn."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    messages: list[Any] = []
    Corrected(
        Imu(angular_velocity=Vector3(z=0.3749)), zero, Kept(messages)
    ).publish()
    assert_that(
        messages[-1].angular_velocity.z,
        close_to(0.37, 1e-12),
        "correction did not preserve the real turn",
    )


def test_corrected_preserves_metadata_and_acceleration() -> None:
    """Correction retains the source header, orientation and acceleration."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    message = Imu(
        header=Header(frame_id="imu_7f"),
        linear_acceleration=Vector3(x=0.17, z=9.807),
        orientation_covariance=[-1.0] + [0.0] * 8,
    )
    messages: list[Any] = []
    Corrected(message, zero, Kept(messages)).publish()
    assert_that(
        (
            messages[-1].header,
            messages[-1].orientation,
            list(messages[-1].orientation_covariance),
            messages[-1].linear_acceleration,
            list(messages[-1].linear_acceleration_covariance),
        ),
        equal_to(
            (
                message.header,
                message.orientation,
                list(message.orientation_covariance),
                message.linear_acceleration,
                list(message.linear_acceleration_covariance),
            )
        ),
        "correction did not preserve IMU metadata and acceleration",
    )


def test_corrected_does_not_change_the_raw_message() -> None:
    """Raw subscribers keep the original biased reading."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0037, -0.0081, 0.0049), 7.13 + index / 100, still=True
        )
    message = Imu(angular_velocity=Vector3(z=0.3749))
    Corrected(message, zero, Kept([])).publish()
    assert_that(
        message.angular_velocity.z,
        equal_to(0.3749),
        "correction did not leave the raw reading intact",
    )


def test_corrected_adds_offset_uncertainty_to_measurement_covariance() -> None:
    """The corrected stream includes uncertainty of the learned offset."""
    zero: Zero = Zeroed(
        (0.97, 80, 0.003), (0.0, 0.0), Averaged((0, (0.0,) * 3, (0.0,) * 3))
    )
    for index in range(101):
        zero = zero.read(
            (0.0, 0.0, 0.005 + 0.001 * (-1) ** index),
            7.13 + index / 100,
            still=True,
        )
    messages: list[Any] = []
    Corrected(
        Imu(
            angular_velocity_covariance=[
                1e-6,
                0.0,
                0.0,
                0.0,
                1e-6,
                0.0,
                0.0,
                0.0,
                1e-6,
            ]
        ),
        zero,
        Kept(messages),
    ).publish()
    assert_that(
        messages[-1].angular_velocity_covariance[8],
        close_to(1e-6 + zero.variance()[2], 1e-15),
        "corrected covariance did not include offset uncertainty",
    )
