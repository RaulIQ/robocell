"""Entry point of robocell."""

import threading
import time
from math import sin

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(
    "/ws/src/robocell_sim/models/waffle_pi/scene.xml"
)
data = mujoco.MjData(model)
pause = threading.Event()


def key(keycode: int) -> None:
    """Toggle the pause when space is pressed in the viewer."""
    if keycode != ord(" "):
        return
    if pause.is_set():
        pause.clear()
    else:
        pause.set()


with mujoco.viewer.launch_passive(model, data, key_callback=key) as viewer:
    while viewer.is_running():
        step_start = time.time()

        if not pause.is_set():
            data.ctrl[model.actuator("wheel_left").id] = 4 + 2 * sin(data.time)
            data.ctrl[model.actuator("wheel_right").id] = 4 - 2 * sin(data.time)
            mujoco.mj_step(model, data)
        viewer.sync()

        # держать реальное время: шаг симуляции = model.opt.timestep
        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
