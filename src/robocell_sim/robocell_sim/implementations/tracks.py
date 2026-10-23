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

    The queue delivers the encoder and true positions in meters together
    with the slip of the wheels, the axes hold the chart, and the lines are
    the encoder track, the true track and the slip marks, in that order.
    The slip marks sit on the encoder positions where the wheels slipped.
    The title tells the latest gap between the two tracks.
    """

    _queue: multiprocessing.queues.Queue[
        tuple[tuple[float, float], tuple[float, float], bool]
    ]
    _axes: Axes
    _lines: tuple[Line2D, Line2D, Line2D]

    @override
    def draw(self) -> None:
        points = [
            self._queue.get(timeout=1) for _ in range(self._queue.qsize())
        ]
        for line, track in zip(
            self._lines,
            (
                [point[0] for point in points],
                [point[1] for point in points],
                [point[0] for point in points if point[2]],
            ),
            strict=True,
        ):
            line.set_data(
                np.append(line.get_xdata(), [point[0] for point in track]),
                np.append(line.get_ydata(), [point[1] for point in track]),
            )
        self._axes.relim()
        self._axes.autoscale_view()
        self._axes.set_title(
            f"Encoders are {math.dist(*points[-1][:2]) * 1000:.0f} mm off"
            if points
            else self._axes.get_title()
        )
