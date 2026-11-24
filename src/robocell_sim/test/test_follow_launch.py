"""Tests for connecting target following to the workshop."""

from pathlib import Path

import yaml
from defusedxml import ElementTree
from hamcrest import assert_that, equal_to


def test_follow_sends_both_command_sources_through_the_mux() -> None:
    """Manual commands take priority over Nav2 before reaching the driver."""
    root = Path(__file__).parents[1]
    launch = ElementTree.parse(root / "launch/follow.launch.xml")
    config = yaml.safe_load((root / "config/follow.yaml").read_text())
    mux = config["twist_mux"]["ros__parameters"]
    assert_that(
        (
            [
                arg.get("value")
                for arg in launch.findall("./include/arg[@name='panel_topic']")
            ],
            [
                remap.get("to")
                for remap in launch.findall(
                    "./node[@name='controller_server']/remap[@from='cmd_vel']"
                )
            ],
            [
                remap.get("to")
                for remap in launch.findall(
                    "./node[@name='twist_mux']/remap[@from='cmd_vel_out']"
                )
            ],
            mux["topics"]["manual"]["priority"]
            > mux["topics"]["navigation"]["priority"],
            mux["use_stamped"],
            config["controller_server"]["ros__parameters"][
                "enable_stamped_cmd_vel"
            ],
        ),
        equal_to(
            (
                ["/cmd_vel_manual"],
                ["/cmd_vel_nav"],
                ["/cmd_vel"],
                True,
                False,
                False,
            )
        ),
        "following did not route manual and navigation commands safely",
    )
