"""Chart of the latest lidar sweep."""

import math
import multiprocessing.queues
from dataclasses import dataclass
from typing import final, override

from matplotlib.lines import Line2D

from robocell_sim.interfaces.chart import Chart


@final
@dataclass(frozen=True)
class Sweep(Chart):
    """Points on a polar chart where the rays of the latest scan echoed.

    The queue delivers scans as ranges in meters, one per degree
    counterclockwise from ahead, and the line holds the echoes of the
    latest of them as angles in radians and ranges. Rays with no echo
    stay off the chart.
    """

    _queue: multiprocessing.queues.Queue[tuple[float, ...]]
    _line: Line2D

    @override
    def draw(self) -> None:
        scans = [self._queue.get(timeout=1) for _ in range(self._queue.qsize())]
        if scans:
            self._line.set_data(
                [
                    math.radians(index)
                    for index, distance in enumerate(scans[-1])
                    if math.isfinite(distance)
                ],
                [distance for distance in scans[-1] if math.isfinite(distance)],
            )
