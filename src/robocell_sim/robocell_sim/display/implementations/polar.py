"""Polar chart window of the lidar scan."""

import contextlib
import multiprocessing.queues
import queue
from dataclasses import dataclass
from typing import final, override

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.projections.polar import PolarAxes

from robocell_sim.display.implementations.sweep import Sweep
from robocell_sim.display.interfaces.radar import Radar


@final
@dataclass(frozen=True)
class Polar(Radar):
    """Tk window that draws the latest lidar scan around the robot.

    The queue carries scans from the simulation to the window, which runs
    in a process of its own, and drops them while it is full, so a slow
    window never holds the simulation back. The reach is the longest range
    in meters the chart shows. Ahead points up and the angles grow
    counterclockwise, so the left of the robot is on the left of the chart.
    """

    _queue: multiprocessing.queues.Queue[tuple[float, ...]]
    _reach: float

    @override
    def mark(self, scan: tuple[float, ...]) -> None:
        with contextlib.suppress(queue.Full):
            self._queue.put_nowait(scan)

    @override
    def show(self) -> None:
        mpl.use("TkAgg")
        with plt.rc_context(
            {
                "figure.facecolor": "#fcfcfb",
                "axes.facecolor": "#fcfcfb",
                "axes.edgecolor": "#c3c2b7",
                "grid.color": "#e1e0d9",
                "grid.linewidth": 0.8,
                "text.color": "#0b0b0b",
                "xtick.color": "#898781",
                "ytick.color": "#898781",
            }
        ):
            figure = plt.figure(figsize=(6, 6), num="Lidar scan")
            axes = PolarAxes(figure, (0.1, 0.1, 0.8, 0.8))
            figure.add_axes(axes)
            axes.set(
                theta_zero_location="N",
                ylim=(0, self._reach),
                title="Lidar scan, ahead is up",
            )
            chart = Sweep(
                self._queue,
                axes.plot(
                    [],
                    [],
                    color="#2a78d6",
                    linestyle="none",
                    marker=".",
                    markersize=4,
                )[0],
            )
            while plt.fignum_exists(figure.number):
                chart.draw()
                plt.pause(0.2)
