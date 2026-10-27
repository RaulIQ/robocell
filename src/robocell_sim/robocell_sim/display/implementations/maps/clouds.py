"""Chart of the two maps that grow from lidar scans."""

import multiprocessing.queues
from dataclasses import dataclass
from typing import final, override

import numpy as np
from matplotlib.lines import Line2D

from robocell_sim.display.interfaces.chart import Chart


@final
@dataclass(frozen=True)
class Clouds(Chart):
    """Two point clouds that keep every scan placed by two poses.

    The queue delivers the points of a scan placed by the true and by the
    reckoned pose, in meters, and the lines hold the map by the true pose
    and the map by the reckoned pose, in that order. Points pile up, so
    the maps grow with every scan that arrives.
    """

    _queue: multiprocessing.queues.Queue[
        tuple[list[tuple[float, float]], list[tuple[float, float]]]
    ]
    _lines: tuple[Line2D, Line2D]

    @override
    def draw(self) -> None:
        for batch in [
            self._queue.get(timeout=1) for _ in range(self._queue.qsize())
        ]:
            for line, points in zip(self._lines, batch, strict=True):
                line.set_data(
                    np.append(line.get_xdata(), [point[0] for point in points]),
                    np.append(line.get_ydata(), [point[1] for point in points]),
                )
