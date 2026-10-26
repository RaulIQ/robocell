"""Tests for the RViz view of the workshop."""

from pathlib import Path

import yaml
from hamcrest import assert_that, equal_to


def test_workshop_rviz_shows_the_true_pose() -> None:
    """An odometry display draws the true pose that MuJoCo publishes."""
    assert_that(
        [
            display["Topic"]["Value"]
            for display in yaml.safe_load(
                (Path(__file__).parents[1] / "rviz/workshop.rviz").read_text()
            )["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/Odometry"
        ],
        equal_to(["/ground_truth/odom"]),
        "RViz did not show the true pose of the robot",
    )


def test_workshop_rviz_shows_the_lidar_scan() -> None:
    """A laser scan display draws the echoes the lidar publishes."""
    assert_that(
        [
            display["Topic"]["Value"]
            for display in yaml.safe_load(
                (Path(__file__).parents[1] / "rviz/workshop.rviz").read_text()
            )["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/LaserScan"
        ],
        equal_to(["/scan"]),
        "RViz did not show the lidar scan",
    )
