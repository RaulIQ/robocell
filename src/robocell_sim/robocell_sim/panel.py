#!/usr/bin/env python3
"""Graphical ROS control with explicit key release and focus handling."""

from __future__ import annotations

import math
import queue
import threading
import time
import tkinter as tk
from contextlib import suppress
from functools import partial
from tkinter import ttk
from typing import TYPE_CHECKING

import rclpy
from action_msgs.msg import GoalStatus  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from geometry_msgs.msg import PoseStamped, Twist, Vector3  # type: ignore[import-untyped]  # ROS generates message modules without types
from nav2_msgs.action import NavigateToPose  # type: ignore[import-untyped]  # Jazzy ships this module without typing information
from rclpy.action.client import ActionClient, ClientGoalHandle
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import try_shutdown

from robocell_sim.robot.implementations.command.held import Held
from robocell_sim.ros.implementations.settings.declared import Declared

if TYPE_CHECKING:
    from rclpy.task import Future

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
window.title("Robocell control")
window.resizable(width=False, height=False)
window.configure(background="#eff3f5")
style = ttk.Style(window)
style.theme_use("clam")
style.configure("TFrame", background="#eff3f5")
style.configure("TLabel", background="#eff3f5", foreground="#183444")
style.configure("Title.TLabel", font=("TkDefaultFont", 20, "bold"))
style.configure("Speed.TLabel", font=("TkDefaultFont", 17, "bold"))
style.configure("TButton", padding=(12, 8))
frame = ttk.Frame(window, padding=24)
frame.grid()
ttk.Label(frame, text="Robocell", style="Title.TLabel").grid(
    row=0, column=0, columnspan=2, sticky="w"
)
ttk.Label(frame, text="Hold arrows to drive · release to stop").grid(
    row=1, column=0, columnspan=2, sticky="w", pady=(4, 20)
)
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
selection = tk.StringVar(window, value="Robot")
selected = [topic]
target: list[tuple[PoseStamped, float]] = []
goals: list[ClientGoalHandle] = []
following = ttk.Label(frame)
focused = threading.Event()
closed = threading.Event()
releases: dict[str, str] = {}


def intent() -> Held:
    """Read the current immutable intent without consuming it."""
    command = commands.get_nowait()
    commands.put(command)
    return command


def display() -> None:
    """Show selected settings and currently requested movement."""
    command = intent()
    linear.configure(text=f"{command.speeds()[0]:.2f} m/s")
    angular.configure(text=f"{command.speeds()[1]:.1f} rad/s")
    if focused.is_set():
        status.configure(
            text=(
                f"Driving · {command.velocity()[0]:+.2f} m/s · "
                f"{command.velocity()[1]:+.1f} rad/s"
                if command.velocity() != (0.0, 0.0)
                else "Ready · hold arrows to drive"
            )
        )


def publish() -> None:
    """Publish the current intent immediately on wall time."""
    command = intent()
    if node.context.ok():
        selected[0].publish(
            Twist(
                linear=Vector3(x=command.velocity()[0]),
                angular=Vector3(z=command.velocity()[1]),
            )
        )
    display()


def press(event: tk.Event[tk.Misc]) -> str:
    """Collapse X11 repeat pairs and accept fresh key presses."""
    key = event.keysym
    if key in releases:
        window.after_cancel(releases.pop(key))
    if focused.is_set():
        commands.put(commands.get_nowait().press(key))
        publish()
    return "break"


def release(event: tk.Event[tk.Misc]) -> str:
    """Delay release until queued X11 repeat presses have been handled."""
    key = event.keysym
    if key in releases:
        window.after_cancel(releases.pop(key))
    releases[key] = window.after_idle(lambda: forget(key))
    return "break"


def forget(key: str) -> None:
    """Release a key once it is known to be a genuine release."""
    releases.pop(key, "")
    commands.put(commands.get_nowait().release(key))
    if focused.is_set():
        publish()


def stop() -> None:
    """Cancel held movement until its keys are released and pressed again."""
    commands.put(commands.get_nowait().press("Escape"))
    publish()


def select() -> None:
    """Stop the previous source before changing the destination."""
    stop()
    selected[0] = (
        cube if cube is not None and selection.get() == "Cube" else topic
    )


def observe_target(message: PoseStamped) -> None:
    """Remember the last cube pose for starting a follow goal."""
    target[:] = [(message, time.monotonic())]


def follow() -> None:
    """Start following the cube or cancel the active follow goal."""
    if goals:
        follow_button.state(["disabled"])
        goals[0].cancel_goal_async().add_done_callback(cancelled)  # type: ignore[no-untyped-call]  # Jazzy action client lacks annotations
    else:
        start()


def start() -> None:
    """Submit the current cube pose when its source and Nav2 are ready."""
    if navigation is None or not target or not navigation.server_is_ready():  # type: ignore[no-untyped-call]  # Jazzy action client lacks annotations
        following.configure(text="Waiting for Nav2 and the cube pose")
        return
    pose, arrival = target[-1]
    if time.monotonic() - arrival >= 1.0:
        following.configure(text="Waiting for a fresh cube pose")
        return
    stop()
    follow_button.state(["disabled"])
    following.configure(text="Starting follow")
    navigation.send_goal_async(
        NavigateToPose.Goal(pose=pose)
    ).add_done_callback(  # type: ignore[no-untyped-call]  # Jazzy action client lacks annotations
        accepted
    )


def accepted(future: Future) -> None:
    """Track the goal handle so the operator can cancel it."""
    follow_button.state(["!disabled"])
    goal = future.result()  # type: ignore[no-untyped-call]  # Jazzy future lacks annotations
    if goal is None or not goal.accepted:
        following.configure(text="Nav2 rejected the cube goal")
        return
    goals.append(goal)
    follow_button.configure(text="Cancel follow")
    following.configure(text="Following cube")
    goal.get_result_async().add_done_callback(finished)


def cancelled(future: Future) -> None:
    """Allow another cancellation attempt if Nav2 rejects the request."""
    response = future.result()  # type: ignore[no-untyped-call]  # Jazzy future lacks annotations
    if goals and (response is None or not response.goals_canceling):
        follow_button.state(["!disabled"])
        following.configure(text="Nav2 did not cancel the goal")


def finished(future: Future) -> None:
    """Show when Nav2 finishes the follow action."""
    goals.clear()
    follow_button.state(["!disabled"])
    follow_button.configure(text="Follow cube")
    response = future.result()  # type: ignore[no-untyped-call]  # Jazzy future lacks annotations
    following.configure(
        text=(
            "Follow canceled"
            if response is not None
            and response.status == GoalStatus.STATUS_CANCELED
            else "Follow ended"
        )
    )


def focus(_event: tk.Event[tk.Misc]) -> None:
    """Check focus after Tk has transferred it between child widgets."""
    window.after_idle(observe)


def observe() -> None:
    """Stop when focus leaves the application without resuming old intent."""
    if closed.is_set():
        return
    if window.focus_displayof() is None:
        focused.clear()
        stop()
        status.configure(text="Stopped · release keys before taking control")
    else:
        focused.set()
    display()


def adjust(key: str) -> None:
    """Treat a speed button as one complete press and release."""
    commands.put(commands.get_nowait().release(key).press(key).release(key))
    publish()


def pulse() -> None:
    """Refresh valid held intent independently of character repeat."""
    if not node.context.ok():
        close()
        return
    rclpy.spin_once(node, timeout_sec=0)
    if focused.is_set() and intent().velocity() != (0.0, 0.0):
        publish()
    window.after(50, pulse)


def close() -> None:
    """Send zero before closing the window and releasing ROS resources."""
    if not closed.is_set():
        stop()
        closed.set()
        window.destroy()


for column, keys in enumerate((("x", "w"), ("c", "e"))):
    buttons = ttk.Frame(frame)
    buttons.grid(row=4, column=column, sticky="w", pady=(0, 16))
    for offset, key in enumerate(keys):
        ttk.Button(
            buttons,
            text=f"{'-' if offset == 0 else '+'}  {key}",
            command=partial(adjust, key),
        ).grid(row=0, column=offset, padx=(0, 6))
ttk.Button(frame, text="STOP · Esc / Space / k", command=stop).grid(
    row=5, column=0, columnspan=2, sticky="ew"
)
if cube is not None:
    node.create_subscription(PoseStamped, "/goal_update", observe_target, 1)
    for column, name in enumerate(("Robot", "Cube")):
        ttk.Radiobutton(
            frame, text=name, value=name, variable=selection, command=select
        ).grid(row=7, column=column, sticky="w", pady=(12, 0))
    follow_button = ttk.Button(frame, text="Follow cube", command=follow)
    follow_button.grid(row=8, column=0, columnspan=2, sticky="ew", pady=(8, 0))
    following.grid(row=9, column=0, columnspan=2, sticky="w")
window.bind("<KeyPress>", press)
window.bind("<KeyRelease>", release)
window.bind("<FocusIn>", focus)
window.bind("<FocusOut>", focus)
window.protocol("WM_DELETE_WINDOW", close)
display()
window.after(50, pulse)
try:
    with suppress(KeyboardInterrupt, ExternalShutdownException):
        window.mainloop()
finally:
    close()
    try_shutdown()
