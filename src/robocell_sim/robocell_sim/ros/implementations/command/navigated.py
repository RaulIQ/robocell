"""Nav2 action lifecycle for an operator-requested target pursuit."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, final, override

from action_msgs.msg import GoalStatus  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from nav2_msgs.action import NavigateToPose  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from rclpy.action.client import ActionClient, ClientGoalHandle

from robocell_sim.robot.implementations.following.pursuit import Pursuit
from robocell_sim.ros.interfaces.following import Following
from robocell_sim.ros.messages import PoseStamped

if TYPE_CHECKING:
    from collections.abc import Callable

    from rclpy.task import Future

    from robocell_sim.display.interfaces.follow_view import FollowView

State = tuple[
    list[tuple[PoseStamped, float]], list[ClientGoalHandle], list[Pursuit]
]


def button(view: FollowView, pursuit: Pursuit) -> None:
    """Label the operator's next action."""
    view.button("Cancel follow" if pursuit.requested() else "Follow cube")


def pending(
    pursuit: list[Pursuit], view: FollowView, stop: Callable[[], None]
) -> None:
    """Release manual intent while the server accepts the goal."""
    stop()
    pursuit[0] = pursuit[0].goal("pending")
    view.enabled(value=False)
    view.status("Starting follow")


def watch(goal: ClientGoalHandle, callback: Callable[[Future], None]) -> None:
    """Subscribe to the result of an accepted navigation action."""
    result: Callable[[], Future] = goal.get_result_async
    complete: Callable[[Callable[[Future], None]], None] = (
        result().add_done_callback
    )
    complete(callback)


def active(state: State, view: FollowView, goal: ClientGoalHandle) -> None:
    """Record the accepted action and update the panel."""
    _, goals, pursuit = state
    goals.append(goal)
    pursuit[0] = pursuit[0].goal("active")
    button(view, pursuit[0])
    view.status("Following cube")


@final
@dataclass(frozen=True)
class Navigated(Following):
    """Start, cancel and complete navigation for the selected target."""

    _navigation: ActionClient
    _state: State
    _view: FollowView
    _controls: tuple[Callable[[], None], Callable[[str], None]]

    @override
    def transition(self) -> None:
        _, goals, pursuit = self._state
        if pursuit[0].decision() == "cancel" and goals:
            pursuit[0] = pursuit[0].goal("pending")
            self._view.enabled(value=False)
            self._view.status(
                "Target lost · waiting"
                if pursuit[0].requested()
                else "Canceling follow"
            )
            goals[0].cancel_goal_async().add_done_callback(self.cancelled)  # type: ignore[no-untyped-call]  # Jazzy action client lacks annotations
        elif pursuit[0].decision() == "start":
            self.start()

    @override
    def start(self) -> None:
        target, _, pursuit = self._state
        if not target or not self._navigation.server_is_ready():  # type: ignore[no-untyped-call]  # Jazzy action client lacks annotations
            self._view.status("Waiting for Nav2 and the cube pose")
            return
        pose, arrival = target[-1]
        if time.monotonic() - arrival >= 1.0:
            self._controls[1]("stale")
            return
        pending(pursuit, self._view, self._controls[0])
        self._navigation.send_goal_async(
            NavigateToPose.Goal(pose=pose)
        ).add_done_callback(self.accepted)  # type: ignore[no-untyped-call]  # Jazzy action client lacks annotations

    @override
    def accepted(self, future: Future) -> None:
        pursuit = self._state[2]
        self._view.enabled(value=True)
        goal = future.result()  # type: ignore[no-untyped-call]  # Jazzy future lacks annotations
        if goal is None or not goal.accepted:
            pursuit[0] = pursuit[0].goal("idle")
            self._view.status("Nav2 rejected the cube goal")
            return
        active(self._state, self._view, goal)
        watch(goal, self.finished)
        self.transition()

    @override
    def cancelled(self, future: Future) -> None:
        _, goals, pursuit = self._state
        response = future.result()  # type: ignore[no-untyped-call]  # Jazzy future lacks annotations
        if goals and (response is None or not response.goals_canceling):
            pursuit[0] = pursuit[0].goal("active")
            self._view.enabled(value=True)
            self._view.status("Nav2 did not cancel the goal")

    @override
    def finished(self, future: Future) -> None:
        _, goals, pursuit = self._state
        goals.clear()
        pursuit[0] = pursuit[0].goal("idle")
        self._view.enabled(value=True)
        button(self._view, pursuit[0])
        response = future.result()  # type: ignore[no-untyped-call]  # Jazzy future lacks annotations
        self._view.status(
            "Follow canceled"
            if response is not None
            and response.status == GoalStatus.STATUS_CANCELED
            else "Follow ended"
        )
        self.transition()
