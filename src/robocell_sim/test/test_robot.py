"""Tests for the robot software that must run without the simulator."""

import re
from pathlib import Path

from hamcrest import assert_that, equal_to


def test_robot_never_imports_mujoco() -> None:
    """No module of the robot software imports MuJoCo."""
    assert_that(
        [
            path.name
            for path in sorted(
                (Path(__file__).parents[1] / "robocell_sim" / "robot").rglob(
                    "*.py"
                )
            )
            if re.search(
                r"^\s*(import|from)\s+mujoco\b", path.read_text(), re.MULTILINE
            )
        ],
        equal_to([]),
        "robot software did not stay free of MuJoCo imports",
    )
