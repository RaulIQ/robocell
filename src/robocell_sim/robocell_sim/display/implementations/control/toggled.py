"""Viewer key bindings that toggle independent simulation flags."""

from dataclasses import dataclass
from threading import Event
from typing import final


@final
@dataclass(frozen=True)
class Toggled:
    """Toggle the event assigned to a viewer key."""

    _keys: dict[int, Event]

    def __call__(self, code: int) -> None:
        """Apply a key press to its configured simulation flag."""
        event = self._keys.get(code)
        if event is not None:
            if event.is_set():
                event.clear()
            else:
                event.set()
