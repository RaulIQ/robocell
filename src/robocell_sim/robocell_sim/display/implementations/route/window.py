"""Window that shows the robot route."""

import multiprocessing.queues
from dataclasses import dataclass
from typing import final, override

import matplotlib as mpl
import matplotlib.pyplot as plt

from robocell_sim.display.implementations.route.tracks import Tracks
from robocell_sim.display.interfaces.route import Route


@final
@dataclass(frozen=True)
class Window(Route):
    """Tk window that redraws the encoder and the true track ten times a second.

    The queue carries the encoder and true positions together with the
    slip of the wheels from the simulation to the window, which runs in a
    process of its own.
    """

    _queue: multiprocessing.queues.Queue[
        tuple[tuple[float, float], tuple[float, float], bool]
    ]

    @override
    def mark(
        self,
        estimate: tuple[float, float],
        truth: tuple[float, float],
        *,
        slipping: bool,
    ) -> None:
        self._queue.put((estimate, truth, slipping))

    @override
    def show(self) -> None:
        mpl.use("TkAgg")
        with plt.rc_context(
            {
                "figure.raise_window": False,
                "figure.facecolor": "#fcfcfb",
                "axes.facecolor": "#fcfcfb",
                "axes.edgecolor": "#c3c2b7",
                "axes.labelcolor": "#52514e",
                "axes.spines.top": False,
                "axes.spines.right": False,
                "axes.grid": True,
                "grid.color": "#e1e0d9",
                "grid.linewidth": 0.8,
                "lines.linewidth": 2,
                "text.color": "#0b0b0b",
                "xtick.color": "#898781",
                "ytick.color": "#898781",
                "legend.frameon": False,
            }
        ):
            figure, axes = plt.subplots(figsize=(6, 6), num="Robot route")
            axes.set(
                xlabel="x, m",
                ylabel="y, m",
                aspect="equal",
                adjustable="datalim",
                title="Waiting for the robot",
            )
            chart = Tracks(
                self._queue,
                axes,
                (
                    axes.plot(
                        [],
                        [],
                        color="#eb6834",
                        linestyle="--",
                        label="Encoders",
                    )[0],
                    axes.plot([], [], color="#2a78d6", label="MuJoCo")[0],
                    axes.plot(
                        [],
                        [],
                        color="#c4312f",
                        linestyle="none",
                        marker="x",
                        label="Slip",
                    )[0],
                ),
            )
            axes.legend(loc="upper left")
            while plt.fignum_exists(figure.number):
                chart.draw()
                plt.pause(0.1)
