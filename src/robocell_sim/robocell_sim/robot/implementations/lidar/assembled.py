"""Full rotations assembled from checked A1 measurements."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.robot.implementations.lidar.circular import Circular
from robocell_sim.robot.interfaces.assembly import Assembly
from robocell_sim.robot.interfaces.sweep import Sweep

_LIMIT = 4096
_READY = 2


@final
@dataclass(frozen=True)
class Assembled(Assembly):
    """Assembly that requires two boundaries before accepting a full turn."""

    _rays: tuple[tuple[float, float, int], ...]
    _phase: tuple[int, int]
    _period: int
    _limits: tuple[float, float]

    @override
    def accept(
        self, echo: tuple[bool, bool, float, float, int], stamp: int
    ) -> tuple[Assembly, tuple[Sweep, ...]]:
        scans: tuple[Sweep, ...] = ()
        rays, phase = (
            (*self._rays, (echo[2], echo[3], echo[4]))
            if self._phase[0]
            else (),
            self._phase,
        )
        if not echo[0]:
            rays, phase = (), (0, 0)
        elif echo[1]:
            scans = (
                (
                    Circular(
                        self._rays,
                        min(
                            self._phase[1],
                            max(
                                0, stamp - len(self._rays) * self._period * 1000
                            ),
                        ),
                        len(self._rays) * self._period / 1e6,
                        self._limits,
                    ),
                )
                if self._phase[0] == _READY and self._rays
                else ()
            )
            rays, phase = (
                ((echo[2], echo[3], echo[4]),),
                (min(_READY, self._phase[0] + 1), stamp),
            )
        if len(rays) > _LIMIT:
            message = "RPLIDAR rotation exceeded its measurement limit"
            raise BufferError(message)
        return (Assembled(rays, phase, self._period, self._limits), scans)
