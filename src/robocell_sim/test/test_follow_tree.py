"""Tests for waiting on camera observations before retrying navigation."""

from pathlib import Path

from defusedxml import ElementTree
from hamcrest import assert_that, equal_to


def test_follow_waits_before_retrying_an_unreachable_goal() -> None:
    """A failed path halts the controller before a bounded wait and retry."""
    tree = ElementTree.parse(Path(__file__).parents[1] / "config/follow.xml")
    recovery = tree.findall("./BehaviorTree/RecoveryNode")[0]
    assert_that(
        (
            recovery.get("number_of_retries"),
            [child.tag for child in recovery],
            recovery.findall("Wait")[0].get("wait_duration"),
        ),
        equal_to(("120", ["PipelineSequence", "Wait"], "1.0")),
        "following did not wait before retrying an unavailable path",
    )


def test_follow_uses_the_waiting_tree() -> None:
    """The follow launch selects the tree that can await new observations."""
    launch = ElementTree.parse(
        Path(__file__).parents[1] / "launch/follow.launch.xml"
    )
    assert_that(
        launch.findall("./arg[@name='follow_tree']")[0].get("default"),
        equal_to("$(find-pkg-share robocell_sim)/config/follow.xml"),
        "the follow launch did not select its waiting behavior",
    )
