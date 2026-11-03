"""Tests for the SLAM configuration of the workshop."""

from pathlib import Path

import yaml
from hamcrest import assert_that, equal_to


def test_slam_uses_driver_scans_and_the_filtered_footprint() -> None:
    """Mapping consumes the driver and EKF frames on simulated time."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/slam.yaml").read_text()
    )["slam_toolbox"]["ros__parameters"]
    assert_that(
        {
            name: config[name]
            for name in (
                "use_sim_time",
                "mode",
                "scan_topic",
                "map_frame",
                "odom_frame",
                "base_frame",
            )
        },
        equal_to(
            {
                "use_sim_time": True,
                "mode": "mapping",
                "scan_topic": "/scan",
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
            }
        ),
        "SLAM did not use the configured scan, clock and footprint frames",
    )


def test_slam_enables_loop_closure_with_the_a1_range() -> None:
    """Mapping closes revisited loops within the emulated A1 range."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/slam.yaml").read_text()
    )["slam_toolbox"]["ros__parameters"]
    assert_that(
        (config["do_loop_closing"], config["max_laser_range"]),
        equal_to((True, 12.0)),
        "SLAM did not enable loop closure within the A1 range",
    )


def test_slam_processes_turns_without_forward_motion() -> None:
    """A turn in place supplies scans even below the distance threshold."""
    assert_that(
        yaml.safe_load(
            (Path(__file__).parents[1] / "config/slam.yaml").read_text()
        )["slam_toolbox"]["ros__parameters"][
            "check_min_dist_and_heading_precisely"
        ],
        equal_to(obj=True),
        "SLAM did not check heading when forward travel was below its limit",
    )
