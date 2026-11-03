"""Tests for stationary evidence from wheel feedback and the IMU."""

from geometry_msgs.msg import Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from hamcrest import assert_that, equal_to
from robocell_sim.ros.implementations.imu.rested import Rested
from sensor_msgs.msg import Imu, JointState  # type: ignore[import-untyped]  # ROS generates message modules without types


def test_rested_accepts_fresh_stationary_wheel_feedback() -> None:
    """Fresh wheel standstill and gravity support stationary calibration."""
    assert_that(
        Rested(
            JointState(name=["port", "starboard"], velocity=[0.001, -0.002]),
            Imu(linear_acceleration=Vector3(z=9.81)),
            (0.02, 0.1),
            ("port", "starboard"),
        ).still(),
        equal_to(obj=True),
        "fresh stationary feedback did not permit calibration",
    )


def test_rested_rejects_missing_wheel_feedback() -> None:
    """No wheel feedback cannot establish that the robot is stationary."""
    assert_that(
        Rested(
            JointState(),
            Imu(linear_acceleration=Vector3(z=9.81)),
            (0.02, 0.1),
            ("port", "starboard"),
        ).still(),
        equal_to(obj=False),
        "missing wheel feedback did not inhibit calibration",
    )


def test_rested_rejects_a_turn_in_place() -> None:
    """Opposite wheel velocities still mean motion."""
    assert_that(
        Rested(
            JointState(name=["port", "starboard"], velocity=[-0.71, 0.71]),
            Imu(linear_acceleration=Vector3(z=9.81)),
            (0.02, 0.1),
            ("port", "starboard"),
        ).still(),
        equal_to(obj=False),
        "opposite wheel motion did not inhibit calibration",
    )


def test_rested_rejects_stale_wheel_feedback() -> None:
    """Old standstill cannot establish current stationary motion."""
    message = Imu(linear_acceleration=Vector3(z=9.81))
    message.header.stamp.sec = 13
    assert_that(
        Rested(
            JointState(name=["port", "starboard"], velocity=[0.0, 0.0]),
            message,
            (0.02, 0.1),
            ("port", "starboard"),
        ).still(),
        equal_to(obj=False),
        "stale wheel feedback did not inhibit calibration",
    )


def test_rested_rejects_nonfinite_wheel_velocities() -> None:
    """Invalid wheel readings cannot establish stationary motion."""
    assert_that(
        Rested(
            JointState(
                name=["port", "starboard"], velocity=[float("nan"), 0.0]
            ),
            Imu(linear_acceleration=Vector3(z=9.81)),
            (0.02, 0.1),
            ("port", "starboard"),
        ).still(),
        equal_to(obj=False),
        "nonfinite wheel feedback did not inhibit calibration",
    )


def test_rested_rejects_dynamic_acceleration() -> None:
    """Strong acceleration cannot establish a stationary IMU."""
    assert_that(
        Rested(
            JointState(name=["port", "starboard"], velocity=[0.0, 0.0]),
            Imu(linear_acceleration=Vector3(z=14.13)),
            (0.02, 0.1),
            ("port", "starboard"),
        ).still(),
        equal_to(obj=False),
        "dynamic acceleration did not inhibit calibration",
    )


def test_rested_rejects_feedback_from_a_future_epoch() -> None:
    """A different wheel-clock epoch cannot establish current standstill."""
    joints = JointState(name=["port", "starboard"], velocity=[0.0, 0.0])
    joints.header.stamp.sec = 17
    assert_that(
        Rested(
            joints,
            Imu(linear_acceleration=Vector3(z=9.81)),
            (0.02, 0.1),
            ("port", "starboard"),
        ).still(),
        equal_to(obj=False),
        "future wheel feedback did not inhibit calibration",
    )
