"""Build the control widgets and connect them to input commands."""

import tkinter as tk
from dataclasses import dataclass
from functools import partial
from tkinter import ttk
from typing import final

from robocell_sim.display.interfaces.keys import Keys


def style(window: tk.Tk) -> None:
    """Configure the panel appearance."""
    window.title("Robocell control")
    window.resizable(width=False, height=False)
    window.configure(background="#eff3f5")
    themed(ttk.Style(window))


def themed(theme: ttk.Style) -> None:
    """Apply the control panel widget theme."""
    theme.theme_use("clam")
    theme.configure("TFrame", background="#eff3f5")
    theme.configure("TLabel", background="#eff3f5", foreground="#183444")
    theme.configure("Title.TLabel", font=("TkDefaultFont", 20, "bold"))
    theme.configure("Speed.TLabel", font=("TkDefaultFont", 17, "bold"))
    theme.configure("TButton", padding=(12, 8))


def structure(window: tk.Tk) -> ttk.Frame:
    """Create the panel frame and its driving instructions."""
    frame = ttk.Frame(window, padding=24)
    frame.grid()
    ttk.Label(frame, text="Robocell", style="Title.TLabel").grid(
        row=0, column=0, columnspan=2, sticky="w"
    )
    ttk.Label(frame, text="Hold arrows to drive · release to stop").grid(
        row=1, column=0, columnspan=2, sticky="w", pady=(4, 20)
    )
    return frame


def readings(frame: ttk.Frame) -> tuple[ttk.Label, ttk.Label, ttk.Label]:
    """Build the speed and status displays."""
    linear = ttk.Label(frame, style="Speed.TLabel")
    linear.grid(row=2, column=0, sticky="w", padx=(0, 28))
    angular = ttk.Label(frame, style="Speed.TLabel")
    angular.grid(row=2, column=1, sticky="w")
    ttk.Label(frame, text="Linear · w / x · ±0.01 m/s").grid(
        row=3, column=0, sticky="w", pady=(4, 16)
    )
    ttk.Label(frame, text="Angular · e / c · ±0.1 rad/s").grid(
        row=3, column=1, sticky="w", pady=(4, 16)
    )
    status = ttk.Label(frame, text="Click this window to take control")
    status.grid(row=6, column=0, columnspan=2, sticky="w", pady=(16, 0))
    return linear, angular, status


def controls(frame: ttk.Frame, keys: Keys) -> None:
    """Install the manual speed and stop controls."""
    for column, shortcuts in enumerate((("x", "w"), ("c", "e"))):
        buttons = ttk.Frame(frame)
        buttons.grid(row=4, column=column, sticky="w", pady=(0, 16))
        for offset, key in enumerate(shortcuts):
            ttk.Button(
                buttons,
                text=f"{'-' if offset == 0 else '+'}  {key}",
                command=partial(keys.adjust, key),
            ).grid(row=0, column=offset, padx=(0, 6))
    ttk.Button(frame, text="STOP · Esc / Space / k", command=keys.stop).grid(
        row=5, column=0, columnspan=2, sticky="ew"
    )


@final
@dataclass(frozen=True)
class Layout:
    """Arrange the widgets in a control panel window."""

    _window: tk.Tk

    def __call__(self) -> tuple[ttk.Frame, ttk.Label, ttk.Label, ttk.Label]:
        """Create the driving layout and return its displays."""
        style(self._window)
        frame = structure(self._window)
        return frame, *readings(frame)
