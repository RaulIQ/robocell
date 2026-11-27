"""Tests for safe following transitions around target loss and cancellation."""

from hamcrest import assert_that, equal_to
from robocell_sim.pursuit import Pursuit


def test_pursuit_resumes_only_with_a_fresh_observation_after_target_loss() -> (
    None
):
    """Loss cancels the active goal and a fresh sighting supplies a new one."""
    lost = Pursuit(_intent="follow", _stage="active", _visible=True).sight(
        visible=False
    )
    assert_that(
        (lost.decision(), lost.goal("idle").sight(visible=True).decision()),
        equal_to(("cancel", "start")),
        "following did not restart after a fresh observation",
    )


def test_pursuit_doesnt_resume_after_manual_cancellation() -> None:
    """An operator cancellation wins over later target observations."""
    assert_that(
        Pursuit(_intent="follow", _stage="active", _visible=True)
        .toggle()
        .goal("idle")
        .sight(visible=True)
        .decision(),
        equal_to("wait"),
        "following did not stay idle after manual cancellation",
    )


def test_pursuit_doesnt_start_duplicates_while_nav2_accepts_a_goal() -> None:
    """Repeated observations cannot submit duplicate goals during startup."""
    assert_that(
        Pursuit(_intent="stop", _stage="idle", _visible=False)
        .toggle()
        .sight(visible=True)
        .goal("pending")
        .decision(),
        equal_to("wait"),
        "following did not wait for Nav2 to accept the pending goal",
    )
