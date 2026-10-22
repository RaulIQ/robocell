"""Chart of the encoder and true tracks."""

import math
import multiprocessing.queues
from dataclasses import dataclass
from typing import final, override

import numpy as np
from matplotlib.axes import Axes
from matplotlib.lines import Line2D

from robocell_sim.interfaces.chart import Chart


@final
@dataclass(frozen=True)
class Tracks(Chart):
    """Two lines on the floor plane, one from the encoders, one from MuJoCo.

    The queue delivers pairs of encoder and true positions in meters, the
    axes hold the chart, and the lines are the encoder and the true track,
    in that order. The title tells the latest gap between them.
    """

    _queue: multiprocessing.queues.Queue[
        tuple[tuple[float, float], tuple[float, float]]
    ]
    _axes: Axes
    _lines: tuple[Line2D, Line2D]

    @override
    def draw(self) -> None:
        points = [
            self._queue.get(timeout=1) for _ in range(self._queue.qsize())
        ]
        for line, track in zip(
            self._lines, zip(*points, strict=True), strict=False
        ):
            line.set_data(
                np.append(line.get_xdata(), [point[0] for point in track]),
                np.append(line.get_ydata(), [point[1] for point in track]),
            )
        self._axes.relim()
        self._axes.autoscale_view()
        self._axes.set_title(
            f"Encoders are {math.dist(*points[-1]) * 1000:.0f} mm off"
            if points
            else self._axes.get_title()
        )
