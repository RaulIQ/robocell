"""Tests for the panel stop command with real Tk event types."""

import tkinter as tk
from queue import SimpleQueue
from threading import Event

from hamcrest import assert_that, equal_to
from robocell_sim.display.implementations.control.intent import Intent
from robocell_sim.display.implementations.control.keyboard import Keyboard
from robocell_sim.robot.implementations.command.held import Held


def test_keyboard_stop_publishes_zero_velocity() -> None:
    """The stop control relinquishes held motion before publishing."""
    commands: SimpleQueue[Held] = SimpleQueue()
    commands.put(Held(frozenset(), frozenset(), 0.17, 0.9).press("Up"))
    motion: list[tuple[float, float]] = []
    Keyboard(
        tk.Tk(useTk=False),
        commands,
        (Event(), {}),
        lambda: motion.append(Intent(commands)().velocity()),
    ).stop()
    assert_that(
        motion,
        equal_to([(0.0, 0.0)]),
        "the stop command did not publish zero velocity",
    )
