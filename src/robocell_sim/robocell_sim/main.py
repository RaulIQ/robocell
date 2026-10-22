"""Entry point of robocell."""

import queue
import threading
import time

import mujoco
import mujoco.viewer

from robocell_sim.implementations.arrows import Arrows
from robocell_sim.implementations.differential import Differential
from robocell_sim.implementations.limited import Limited
from robocell_sim.implementations.stepped import Stepped
from robocell_sim.interfaces.command import Command

model = mujoco.MjModel.from_xml_path(
    "/ws/src/robocell_sim/models/waffle_pi/scene.xml"
)
data = mujoco.MjData(model)
pause = threading.Event()
keys: queue.SimpleQueue[int] = queue.SimpleQueue()


def key(keycode: int) -> None:
    """Toggle the pause on space and queue every other key for the drive."""
    if keycode != ord(" "):
        keys.put(keycode)
        return
    if pause.is_set():
        pause.clear()
    else:
        pause.set()


drive = Limited(Differential(0.033, 0.288), 7.88)
command: Command = Arrows(Stepped(0.05, 5, 0), Stepped(0.3, 6, 0))

with mujoco.viewer.launch_passive(model, data, key_callback=key) as viewer:
    while viewer.is_running():
        step_start = time.time()

        while not keys.empty():
            command = command.press(keys.get())
        if not pause.is_set():
            data.ctrl[
                [
                    model.actuator("wheel_left").id,
                    model.actuator("wheel_right").id,
                ]
            ] = drive.wheels(*command.velocity())
            mujoco.mj_step(model, data)
        viewer.sync()

        # держать реальное время: шаг симуляции = model.opt.timestep
        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
