"""Tests for the workshop mapping launch."""

from pathlib import Path

from defusedxml import ElementTree
from hamcrest import assert_that, equal_to


def test_mapping_launch_removes_the_static_parent_of_odom() -> None:
    """SLAM owns map to odom without the workshop world to odom link."""
    assert_that(
        [
            argument.get("value")
            for include in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("include")
            if include.get("file", "").endswith("workshop.launch.xml")
            for argument in include.iter("arg")
            if argument.get("name") == "fixed_odom"
        ],
        equal_to(["false"]),
        "mapping did not remove the static parent of odom",
    )


def test_mapping_launch_activates_slam_on_simulated_time() -> None:
    """The upstream launch configures and activates the mapping lifecycle."""
    assert_that(
        [
            {
                argument.get("name"): argument.get("value")
                for argument in include.iter("arg")
            }
            for include in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("include")
            if include.get("file", "").endswith("online_async_launch.py")
        ],
        equal_to(
            [
                {
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                    "use_sim_time": "true",
                    "slam_params_file": (
                        "$(find-pkg-share robocell_sim)/config/slam.yaml"
                    ),
                }
            ]
        ),
        "mapping did not activate SLAM with its simulated-time configuration",
    )


def test_mapping_launch_selects_the_map_view() -> None:
    """RViz uses map as its fixed frame in mapping mode."""
    assert_that(
        [
            argument.get("value")
            for include in ElementTree.parse(
                Path(__file__).parents[1] / "launch/mapping.launch.xml"
            ).iter("include")
            for argument in include.iter("arg")
            if argument.get("name") == "rviz_config"
        ],
        equal_to(["$(find-pkg-share robocell_sim)/rviz/mapping.rviz"]),
        "mapping did not select the map view",
    )
