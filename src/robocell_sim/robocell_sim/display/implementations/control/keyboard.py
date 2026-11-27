"""Manual keyboard handling for the control panel."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, final, override

from robocell_sim.display.interfaces.keys import Keys

if TYPE_CHECKING:
    import tkinter as tk
    from collections.abc import Callable
    from queue import SimpleQueue
    from threading import Event

    from robocell_sim.robot.implementations.command.held import Held


@final
@dataclass(frozen=True)
class Keyboard(Keys):
    """Track held keys and preserve focus and repeat semantics."""

    _window: tk.Tk
    _commands: SimpleQueue[Held]
    _state: tuple[Event, dict[str, str]]
    _publish: Callable[[], None]

    @override
    def press(self, event: tk.Event[tk.Misc]) -> str:
        """Collapse X11 repeat pairs and accept fresh key presses."""
        key = event.keysym
        if key in self._state[1]:
            self._window.after_cancel(self._state[1].pop(key))
        if self._state[0].is_set():
            self._commands.put(self._commands.get_nowait().press(key))
            self._publish()
        return "break"

    @override
    def release(self, event: tk.Event[tk.Misc]) -> str:
        """Delay release until queued X11 repeat presses have been handled."""
        key = event.keysym
        if key in self._state[1]:
            self._window.after_cancel(self._state[1].pop(key))
        self._state[1][key] = self._window.after_idle(lambda: self.forget(key))
        return "break"

    @override
    def forget(self, key: str) -> None:
        """Release a key once it is known to be a genuine release."""
        self._state[1].pop(key, "")
        self._commands.put(self._commands.get_nowait().release(key))
        if self._state[0].is_set():
            self._publish()

    @override
    def stop(self) -> None:
        """Cancel movement until the keys are released and pressed again."""
        self._commands.put(self._commands.get_nowait().press("Escape"))
        self._publish()

    @override
    def adjust(self, key: str) -> None:
        """Treat a speed button as one complete press and release."""
        self._commands.put(
            self._commands.get_nowait().release(key).press(key).release(key)
        )
        self._publish()
