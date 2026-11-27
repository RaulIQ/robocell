"""Track the selected target and operator follow request."""

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import final, override

from robocell_sim.display.interfaces.follow_view import FollowView
from robocell_sim.robot.implementations.following.pursuit import Pursuit
from robocell_sim.ros.interfaces.follow_request import FollowRequest, String
from robocell_sim.ros.messages import PoseStamped

_TIMEOUT = 1.0


@final
@dataclass(frozen=True)
class Requested(FollowRequest):
    """Update the current target and notify the action controller."""

    _target: list[tuple[PoseStamped, float]]
    _pursuit: list[Pursuit]
    _view: FollowView
    _transition: Callable[[], None]

    @override
    def observe(self, message: PoseStamped) -> None:
        self._target[:] = [(message, time.monotonic())]
        self._pursuit[0] = self._pursuit[0].sight(visible=True)
        if not self._pursuit[0].requested():
            self._view.status("Target ready")
        self._transition()

    @override
    def status(self, message: String) -> None:
        if message.data == "detected":
            if not self._pursuit[0].requested():
                self._view.status("Target detected")
        else:
            self.lose(message.data)

    @override
    def lose(self, reason: str) -> None:
        self._target.clear()
        self._pursuit[0] = self._pursuit[0].sight(visible=False)
        suffix = " · waiting" if self._pursuit[0].requested() else ""
        self._view.status(f"Target {reason}{suffix}")
        self._transition()

    @override
    def toggle(self) -> None:
        pursuit = self._pursuit[0].toggle()
        self._pursuit[0] = pursuit
        self._view.button(
            "Cancel follow" if pursuit.requested() else "Follow cube"
        )
        if pursuit.requested():
            self._view.status("Waiting for a fresh cube pose")
        self._transition()

    @override
    def expire(self) -> None:
        if self._target and time.monotonic() - self._target[-1][1] >= _TIMEOUT:
            self.lose("stale")
