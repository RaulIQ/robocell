"""Tests for the SLAM configuration of the workshop."""

from pathlib import Path

import yaml
from defusedxml import ElementTree
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
                "scan_topic": "/scan/slam",
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
            }
        ),
        "SLAM did not use the configured scan, clock and footprint frames",
    )


def test_slam_uses_the_scan_range_and_closes_loops() -> None:
    """SLAM takes its range from the scan and matches revisited places."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/slam.yaml").read_text()
    )["slam_toolbox"]["ros__parameters"]
    assert_that(
        (config["do_loop_closing"], config["max_laser_range"]),
        equal_to((True, -1.0)),
        "SLAM did not close loops using the reported scan range",
    )


def test_workshop_defaults_to_the_shared_lidar_range() -> None:
    """Workshop supplies the default for both launch modes."""
    assert_that(
        [
            arg.get("default")
            for arg in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).findall("./arg[@name='lidar_range_max']")
        ],
        equal_to(["4.25"]),
        "Workshop did not default to a 4.25 m lidar range",
    )


def test_workshop_shares_the_lidar_range() -> None:
    """The simulator and driver receive one launch setting for range."""
    launch = ElementTree.parse(
        Path(__file__).parents[1] / "launch/workshop.launch.xml"
    )
    assert_that(
        (
            "--lidar-range-max=$(var lidar_range_max)"
            in (
                " ".join(
                    node.get("args", "")
                    for node in launch.findall("./node[@exec='main.py']")
                )
            ),
            [
                item.get("value")
                for item in launch.findall(
                    "./node[@exec='lidar.py']/param[@name='range_max']"
                )
            ],
        ),
        equal_to((True, ["$(var lidar_range_max)"])),
        "The simulator and driver did not receive the same range setting",
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


def test_slam_separates_local_links_from_workshop_loop_search() -> None:
    """Room-wide scan centroids cannot hide a revisited robot trajectory."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/slam.yaml").read_text()
    )["slam_toolbox"]["ros__parameters"]
    assert_that(
        (
            config["use_scan_barycenter"],
            config["link_scan_maximum_distance"],
            config["loop_search_maximum_distance"],
        ),
        equal_to((False, 0.4, 1.2)),
        "SLAM did not separate nearby links from revisited robot poses",
    )
