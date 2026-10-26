"""Tests for the launch file of the workshop."""

from pathlib import Path

from defusedxml import ElementTree
from hamcrest import assert_that, equal_to


def test_workshop_launch_fixes_odom_in_the_world() -> None:
    """A static transform places odom in the world frame of MuJoCo."""
    assert_that(
        [
            node.get("args", "")
            for node in ElementTree.parse(
                Path(__file__).parents[1] / "launch/workshop.launch.xml"
            ).iter("node")
            if node.get("exec") == "static_transform_publisher"
        ],
        equal_to(["--frame-id world --child-frame-id odom"]),
        "launch file did not place odom in the world frame",
    )
