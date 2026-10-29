"""Tests for the planar fusion of wheel odometry and the IMU."""

from pathlib import Path

import yaml
from hamcrest import assert_that, equal_to


def test_ekf_fuses_wheel_velocities_without_reusing_their_pose() -> None:
    """The filter uses independent wheel velocities instead of derived pose."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/ekf.yaml").read_text()
    )["ekf_filter_node"]["ros__parameters"]
    assert_that(
        (
            config["odom0"],
            [
                index
                for index, value in enumerate(config["odom0_config"])
                if value
            ],
        ),
        equal_to(("/wheel/odometry", [6, 11])),
        "filter did not select only forward and yaw velocities from the wheels",
    )


def test_ekf_uses_the_gyro_without_orientation_or_acceleration() -> None:
    """The IMU supplies yaw rate without an orientation or gravity removal."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/ekf.yaml").read_text()
    )["ekf_filter_node"]["ros__parameters"]
    assert_that(
        (
            config["imu0"],
            [
                index
                for index, value in enumerate(config["imu0_config"])
                if value
            ],
        ),
        equal_to(("/imu", [11])),
        "filter did not select only the yaw rate from the IMU",
    )


def test_ekf_publishes_the_planar_footprint_transform() -> None:
    """The filtered pose owns odom to base_footprint on simulated time."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/ekf.yaml").read_text()
    )["ekf_filter_node"]["ros__parameters"]
    assert_that(
        {
            name: config[name]
            for name in (
                "use_sim_time",
                "two_d_mode",
                "publish_tf",
                "odom_frame",
                "world_frame",
                "base_link_frame",
            )
        },
        equal_to(
            {
                "use_sim_time": True,
                "two_d_mode": True,
                "publish_tf": True,
                "odom_frame": "odom",
                "world_frame": "odom",
                "base_link_frame": "base_footprint",
            }
        ),
        "filter did not publish the planar footprint transform "
        "on simulated time",
    )
