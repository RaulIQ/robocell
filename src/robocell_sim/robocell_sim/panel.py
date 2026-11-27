#!/usr/bin/env python3
"""Graphical ROS control with explicit key release and focus handling."""

from __future__ import annotations

import math
import queue
import threading
import tkinter as tk
from contextlib import suppress
from tkinter import ttk
from typing import TYPE_CHECKING

import rclpy
from nav2_msgs.action import NavigateToPose  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown

from robocell_sim.display.implementations.control.follow_view import Labeled
from robocell_sim.display.implementations.control.intent import Intent
from robocell_sim.display.implementations.control.keyboard import Keyboard
from robocell_sim.display.implementations.control.layout import Layout, controls
from robocell_sim.display.implementations.control.readings import Readings
from robocell_sim.robot.implementations.command.held import Held
from robocell_sim.robot.implementations.following.pursuit import Pursuit
from robocell_sim.ros.implementations.command.manual import Manual, Twist
from robocell_sim.ros.implementations.command.navigated import Navigated
from robocell_sim.ros.implementations.command.requested import Requested
from robocell_sim.ros.implementations.settings.declared import Declared
from robocell_sim.ros.interfaces.follow_request import String
from robocell_sim.ros.messages import PoseStamped

if TYPE_CHECKING:
    from robocell_sim.ros.interfaces.topic import Topic


_LINEAR = (0.01, 0.26)
_ANGULAR = (0.1, 1.8)


rclpy.init()
node = rclpy.create_node("control_panel")
settings = Declared(node)
speed = settings.number("speed", _LINEAR[1])
turn = settings.number("turn", _ANGULAR[1])
if not math.isfinite(speed) or not _LINEAR[0] <= speed <= _LINEAR[1]:
    error = "Control panel speed must be between 0.01 and 0.26 m/s"
    raise ValueError(error)
if not math.isfinite(turn) or not _ANGULAR[0] <= turn <= _ANGULAR[1]:
    error = "Control panel turn must be between 0.1 and 1.8 rad/s"
    raise ValueError(error)
commands: queue.SimpleQueue[Held] = queue.SimpleQueue()
commands.put(Held(frozenset(), frozenset(), speed, turn))
topic = node.create_publisher(Twist, "/cmd_vel", 1)
cube = (
    node.create_publisher(Twist, "/cube/cmd_vel", 1)
    if settings.flag("fake_target", default=False)
    else None
)
navigation = (
    ActionClient(node, NavigateToPose, "/navigate_to_pose")  # type: ignore[no-untyped-call]  # Jazzy action client lacks annotations
    if cube is not None
    else None
)
window = tk.Tk()
frame, linear, angular, status = Layout(window)()
selection = tk.StringVar(window, value="Robot")
selected: list[Topic] = [topic]
target: list[tuple[PoseStamped, float]] = []
goals: list[ClientGoalHandle] = []
pursuit = [Pursuit(_intent="stop", _stage="idle", _visible=False)]
following = ttk.Label(frame)
actions: list[Navigated] = []
requests: list[Requested] = []
focused = threading.Event()
closed = threading.Event()
releases: dict[str, str] = {}


intent = Intent(commands)
readings = Readings((linear, angular, status), focused)
manual = Manual(node.context, selected)


def publish() -> None:
    """Send and display the current command."""
    manual(intent().velocity())
    readings(intent())


def select() -> None:
    """Stop the previous source before changing the destination."""
    keys.stop()
    selected[0] = (
        cube if cube is not None and selection.get() == "Cube" else topic
    )


def transition() -> None:
    """Apply the new follow state when the Nav2 client is present."""
    if actions:
        actions[0].transition()


def focus(_event: tk.Event[tk.Misc]) -> None:
    """Check focus after Tk has transferred it between child widgets."""
    window.after_idle(observe)


def observe() -> None:
    """Stop when focus leaves the application without resuming old intent."""
    if closed.is_set():
        return
    if window.focus_displayof() is None:
        focused.clear()
        keys.stop()
        status.configure(text="Stopped · release keys before taking control")
    else:
        focused.set()
    readings(intent())


def pulse() -> None:
    """Refresh valid held intent independently of character repeat."""
    if not node.context.ok():
        close()
        return
    rclpy.spin_once(node, timeout_sec=0)
    if requests:
        requests[0].expire()
    if focused.is_set() and intent().velocity() != (0.0, 0.0):
        publish()
    window.after(50, pulse)


def close() -> None:
    """Send zero before closing the window and releasing ROS resources."""
    if not closed.is_set():
        keys.stop()
        closed.set()
        window.destroy()


keys = Keyboard(window, commands, (focused, releases), publish)
controls(frame, keys)
if cube is not None:
    for column, name in enumerate(("Robot", "Cube")):
        ttk.Radiobutton(
            frame, text=name, value=name, variable=selection, command=select
        ).grid(row=7, column=column, sticky="w", pady=(12, 0))
    follow_button = ttk.Button(frame, text="Follow cube")
    view = Labeled(following, follow_button)
    requested = Requested(target, pursuit, view, transition)
    requests.append(requested)
    follow_button.configure(command=requested.toggle)
    node.create_subscription(PoseStamped, "/goal_update", requested.observe, 1)
    node.create_subscription(String, "/target/status", requested.status, 1)
    if navigation is not None:
        actions.append(
            Navigated(
                navigation,
                (target, goals, pursuit),
                view,
                (keys.stop, requested.lose),
            )
        )
    follow_button.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(8, 0))
    following.grid(row=9, column=0, columnspan=2, sticky="w")
window.bind("<KeyPress>", keys.press)
window.bind("<KeyRelease>", keys.release)
window.bind("<FocusIn>", focus)
window.bind("<FocusOut>", focus)
window.protocol("WM_DELETE_WINDOW", close)
readings(intent())
window.after(50, pulse)
try:
    with suppress(KeyboardInterrupt, ExternalShutdownException):
        window.mainloop()
finally:
    close()
    try_shutdown()
