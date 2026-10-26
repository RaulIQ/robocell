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


def test_workshop_rviz_shows_the_camera_picture() -> None:
    """An image display shows what the robot camera sees, best effort."""
    assert_that(
        [
            (display["Topic"]["Value"], display["Topic"]["Reliability Policy"])
            for display in yaml.safe_load(
                (Path(__file__).parents[1] / "rviz/workshop.rviz").read_text()
            )["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/Image"
        ],
        equal_to([("/camera/image_raw", "Best Effort")]),
        "RViz did not show the camera picture best effort",
    )


def test_workshop_rviz_overlays_the_scene_on_the_camera_picture() -> None:
    """A camera display draws the scene over the picture it receives."""
    assert_that(
        [
            (
                display["Topic"]["Value"],
                display["Topic"]["Reliability Policy"],
                display["Image Rendering"],
            )
            for display in yaml.safe_load(
                (Path(__file__).parents[1] / "rviz/workshop.rviz").read_text()
            )["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/Camera"
        ],
        equal_to(
            [("/camera/image_raw", "Best Effort", "background and overlay")]
        ),
        "RViz did not overlay the scene on the camera picture",
    )


def test_workshop_rviz_overlays_the_lidar_frames_and_robot_on_the_camera() -> (
    None
):
    """The camera overlay shows the lidar, the TF axes and the robot alone."""
    assert_that(
        [
            sorted(
                name
                for name, shown in display["Visibility"].items()
                if name != "Value" and shown
            )
            for display in yaml.safe_load(
                (Path(__file__).parents[1] / "rviz/workshop.rviz").read_text()
            )["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/Camera"
        ],
        equal_to([["Lidar", "RobotModel", "TF"]]),
        "camera overlay did not show the lidar, the frames and the robot",
    )
