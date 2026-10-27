"""Tests for the hardware stand-ins that must run without ROS 2."""

import re
from pathlib import Path

from hamcrest import assert_that, equal_to


def test_hardware_never_imports_ros() -> None:
    """No module of the hardware stand-ins imports rclpy or ROS messages."""
    assert_that(
        [
            path.name
            for path in sorted(
                (Path(__file__).parents[1] / "robocell_sim" / "hardware").rglob(
                    "*.py"
                )
            )
            if re.search(
                r"^\s*(import|from)\s+(rclpy|\w+_(msgs|interfaces))\b",
                path.read_text(),
                re.MULTILINE,
            )
        ],
        equal_to([]),
        "hardware stand-ins did not stay free of ROS imports",
    )
