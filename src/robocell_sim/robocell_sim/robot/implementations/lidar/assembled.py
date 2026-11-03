"""Full rotations assembled from checked A1 measurements."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.lidar.circular import Circular
from robocell_sim.robot.interfaces.assembly import Assembly
from robocell_sim.robot.interfaces.chronology import Chronology
from robocell_sim.robot.interfaces.sweep import Sweep

_LIMIT = 4096
_READY = 2


@final
@dataclass(frozen=True)
class Assembled(Assembly):
    """Assembly that requires two boundaries before accepting a full turn."""

    _rays: tuple[tuple[float, float, int], ...]
    _phase: tuple[int, int]
    _profile: tuple[int, tuple[float, float]]
    _chronology: Chronology

    @override
    def accept(
        self,
        echo: tuple[bool, bool, float, float, int],
        stamp: int,
        ceiling: int,
    ) -> tuple[Assembly, tuple[Sweep, ...]]:
        if echo[0] and echo[1]:
            chronology, stamps = (
                self._chronology.read(
                    min(
                        self._phase[1],
                        max(
                            0,
                            stamp - len(self._rays) * self._profile[0] * 1000,
                        ),
                    ),
                    len(self._rays) * self._profile[0] * 1000,
                    ceiling,
                )
                if self._phase[0] == _READY and self._rays
                else (self._chronology, ())
            )
            return (
                Assembled(
                    ((echo[2], echo[3], echo[4]),),
                    (min(_READY, self._phase[0] + 1), stamp),
                    self._profile,
                    chronology,
                ),
                tuple(
                    Circular(
                        self._rays,
                        beginning,
                        len(self._rays) * self._profile[0] / 1e6,
                        self._profile[1],
                    )
                    for beginning in stamps
                ),
            )
        if echo[0] and len(self._rays) >= _LIMIT:
            message = "RPLIDAR rotation exceeded its measurement limit"
            raise BufferError(message)
        return (
            Assembled(
                (*self._rays, (echo[2], echo[3], echo[4]))
                if echo[0] and self._phase[0]
                else (),
                self._phase if echo[0] else (0, 0),
                self._profile,
                self._chronology if echo[0] else self._chronology.reset(),
            ),
            (),
        )
