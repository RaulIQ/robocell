"""Tests for the RViz mapping view."""

from pathlib import Path

import yaml
from hamcrest import assert_that, equal_to


def test_mapping_rviz_uses_the_map_frame() -> None:
    """The SLAM map defines the frame of the mapping view."""
    assert_that(
        yaml.safe_load(
            (Path(__file__).parents[1] / "rviz/mapping.rviz").read_text()
        )["Visualization Manager"]["Global Options"]["Fixed Frame"],
        equal_to("map"),
        "mapping view did not use the map frame",
    )


def test_mapping_rviz_receives_the_latched_occupancy_map() -> None:
    """A late RViz connection receives the latest occupancy map."""
    assert_that(
        [
            display["Topic"]
            for display in yaml.safe_load(
                (Path(__file__).parents[1] / "rviz/mapping.rviz").read_text()
            )["Visualization Manager"]["Displays"]
            if display["Class"] == "rviz_default_plugins/Map"
        ],
        equal_to(
            [
                {
                    "Value": "/map",
                    "Durability Policy": "Transient Local",
                    "Reliability Policy": "Reliable",
                }
            ]
        ),
        "mapping view did not receive the latched occupancy map",
    )
