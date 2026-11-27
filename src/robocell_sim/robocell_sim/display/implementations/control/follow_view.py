"""Follow status rendered in a Tk control panel."""

from dataclasses import dataclass
from tkinter import ttk
from typing import final, override

from robocell_sim.display.interfaces.follow_view import FollowView


@final
@dataclass(frozen=True)
class Labeled(FollowView):
    """Display follow status using the panel's label and button."""

    _status: ttk.Label
    _button: ttk.Button

    @override
    def status(self, text: str) -> None:
        self._status.configure(text=text)

    @override
    def button(self, text: str) -> None:
        self._button.configure(text=text)

    @override
    def enabled(self, *, value: bool) -> None:
        self._button.state(["!disabled" if value else "disabled"])
