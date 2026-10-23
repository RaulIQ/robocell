"""Window that draws the maps by true and reckoned pose."""

import multiprocessing.queues
from dataclasses import dataclass
from typing import final, override

import matplotlib as mpl
import matplotlib.pyplot as plt

from robocell_sim.display.implementations.clouds import Clouds
from robocell_sim.display.interfaces.atlas import Atlas


@final
@dataclass(frozen=True)
class Survey(Atlas):
    """Tk window with the map by the true pose beside the reckoned one.

    The queue carries the points of each scan placed by both poses from
    the simulation to the window, which runs in a process of its own. Both
    maps share the room axes, so a drift of the reckoned pose shows as
    walls that blur, double or shift on the right.
    """

    _queue: multiprocessing.queues.Queue[
        tuple[list[tuple[float, float]], list[tuple[float, float]]]
    ]

    @override
    def mark(
        self,
        truth: list[tuple[float, float]],
        estimate: list[tuple[float, float]],
    ) -> None:
        self._queue.put((truth, estimate))

    @override
    def show(self) -> None:
        mpl.use("TkAgg")
        with plt.rc_context(
            {
                "figure.facecolor": "#fcfcfb",
                "axes.facecolor": "#fcfcfb",
                "axes.edgecolor": "#c3c2b7",
                "axes.labelcolor": "#52514e",
                "axes.grid": True,
                "grid.color": "#e1e0d9",
                "grid.linewidth": 0.8,
                "text.color": "#0b0b0b",
                "xtick.color": "#898781",
                "ytick.color": "#898781",
            }
        ):
            figure, axes = plt.subplots(
                1,
                2,
                figsize=(11, 5.5),
                num="Lidar map",
                sharex=True,
                sharey=True,
            )
            for side, title in zip(
                axes, ("By the true pose", "By odometry"), strict=True
            ):
                side.set(
                    xlabel="x, m",
                    ylabel="y, m",
                    aspect="equal",
                    xlim=(-3.3, 3.3),
                    ylim=(-3.3, 3.3),
                    title=title,
                )
            chart = Clouds(
                self._queue,
                (
                    axes[0].plot(
                        [],
                        [],
                        color="#2a78d6",
                        linestyle="none",
                        marker=".",
                        markersize=1,
                    )[0],
                    axes[1].plot(
                        [],
                        [],
                        color="#eb6834",
                        linestyle="none",
                        marker=".",
                        markersize=1,
                    )[0],
                ),
            )
            while plt.fignum_exists(figure.number):
                chart.draw()
                plt.pause(0.5)
