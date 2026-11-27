"""Wheel odometry updates driven by encoder acquisitions."""

from collections.abc import Iterator
from dataclasses import dataclass
from queue import SimpleQueue
from typing import final, override

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.interfaces.travel import Travel
from robocell_sim.ros.implementations.odometry.estimated import Estimated
from robocell_sim.ros.interfaces.topic import Topic
from robocell_sim.ros.messages import Header


def pending(
    counts: SimpleQueue[tuple[int, int, float, Header]],
) -> Iterator[tuple[int, int, float, Header]]:
    """Yield each pending encoder reading in order."""
    while not counts.empty():
        yield counts.get()


@final
@dataclass(frozen=True)
class Updated(Report):
    """Integrate queued wheel readings and publish odometry for each stamp."""

    _counts: SimpleQueue[tuple[int, int, float, Header]]
    _travel: list[Travel]
    _frame: str
    _output: tuple[Topic, tuple[float, float]]

    @override
    def publish(self) -> None:
        topic, variances = self._output
        for left, right, moment, header in pending(self._counts):
            self._travel[0] = self._travel[0].read(left, right, moment)
            Estimated(
                self._travel[0], variances, (header, self._frame), topic
            ).publish()
