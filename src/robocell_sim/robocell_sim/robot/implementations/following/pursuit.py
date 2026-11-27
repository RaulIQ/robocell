"""State transitions for safe target following."""

from dataclasses import dataclass, replace
from typing import Literal, final, override

from robocell_sim.robot.interfaces.pursuing import Decision, Pursuing, Stage


@final
@dataclass(frozen=True, kw_only=True)
class Pursuit(Pursuing):
    """Coordinate operator intent, goal lifetime and target availability."""

    _intent: Literal["follow", "stop"]
    _stage: Stage
    _visible: bool

    @override
    def toggle(self) -> "Pursuit":
        return replace(self, _intent="stop" if self.requested() else "follow")

    @override
    def sight(self, *, visible: bool) -> "Pursuit":
        return replace(self, _visible=visible)

    @override
    def goal(self, stage: Stage) -> "Pursuit":
        return replace(self, _stage=stage)

    @override
    def decision(self) -> Decision:
        wanted = self.requested() and self._visible
        result: Decision = "wait"
        if self._stage == "idle" and wanted:
            result = "start"
        elif self._stage == "active" and not wanted:
            result = "cancel"
        return result

    @override
    def requested(self) -> bool:
        return self._intent == "follow"
