"""Current manual-driving settings shown in the panel."""

from dataclasses import dataclass
from threading import Event
from tkinter import ttk
from typing import final

from robocell_sim.robot.implementations.command.held import Held


@final
@dataclass(frozen=True)
class Readings:
    """Render held drive intent and highlight movement while focused."""

    _labels: tuple[ttk.Label, ttk.Label, ttk.Label]
    _focused: Event

    def __call__(self, command: Held) -> None:
        """Refresh the speed and direction readouts."""
        linear, angular, status = self._labels
        speeds, velocity = command.speeds(), command.velocity()
        linear.configure(text=f"{speeds[0]:.2f} m/s")
        angular.configure(text=f"{speeds[1]:.1f} rad/s")
        if self._focused.is_set():
            label = (
                f"Driving · {velocity[0]:+.2f} m/s · {velocity[1]:+.1f} rad/s"
                if velocity != (0.0, 0.0)
                else "Ready · hold arrows to drive"
            )
            status.configure(text=label)
