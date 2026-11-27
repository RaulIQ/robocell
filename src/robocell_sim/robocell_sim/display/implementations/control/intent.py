"""A nondestructive read of a held-key command."""

from dataclasses import dataclass
from queue import SimpleQueue
from typing import final

from robocell_sim.robot.implementations.command.held import Held


@final
@dataclass(frozen=True)
class Intent:
    """Read the current immutable keyboard intent between events."""

    _commands: SimpleQueue[Held]

    def __call__(self) -> Held:
        """Return the current command without consuming it."""
        command = self._commands.get_nowait()
        self._commands.put(command)
        return command
