"""Hello world entry point of robocell."""

import time
from math import sin

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(
    "/ws/src/robocell_sim/models/panda/scene.xml"
)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        data.ctrl[0] = 0.5 * sin(data.time)

        mujoco.mj_step(model, data)
        viewer.sync()

        # держать реальное время: шаг симуляции = model.opt.timestep
        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
