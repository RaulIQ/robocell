"""Direction and speed selection without operating-system key repeat."""

from dataclasses import dataclass, replace
from typing import final, override

from robocell_sim.robot.interfaces.manual import Manual


@final
@dataclass(frozen=True)
class Held(Manual):
    """Manual intent that inhibits cancelled keys until their release."""

    _keys: frozenset[str]
    _blocked: frozenset[str]
    _linear: float
    _angular: float

    @override
    def press(self, key: str) -> "Held":
        result = self
        if key in {"Escape", "space", "k"}:
            result = replace(
                self, _keys=frozenset(), _blocked=self._blocked | self._keys
            )
        elif (
            key in {"Up", "Down", "Left", "Right", "w", "x", "e", "c"}
            and key not in self._keys | self._blocked
        ):
            result = replace(
                self,
                _keys=self._keys | {key},
                _linear=round(
                    min(
                        0.26,
                        max(0.01, self._linear + {"w": 0.01, "x": -0.01}[key]),
                    ),
                    6,
                )
                if key in {"w", "x"}
                else self._linear,
                _angular=round(
                    min(
                        1.8,
                        max(0.1, self._angular + {"e": 0.1, "c": -0.1}[key]),
                    ),
                    6,
                )
                if key in {"e", "c"}
                else self._angular,
            )
        return result

    @override
    def release(self, key: str) -> "Held":
        return replace(
            self, _keys=self._keys - {key}, _blocked=self._blocked - {key}
        )

    @override
    def velocity(self) -> tuple[float, float]:
        return (
            self._linear * (("Up" in self._keys) - ("Down" in self._keys)),
            self._angular * (("Left" in self._keys) - ("Right" in self._keys)),
        )

    @override
    def speeds(self) -> tuple[float, float]:
        return (self._linear, self._angular)
