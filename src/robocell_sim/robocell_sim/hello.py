"""Hello world entry point of robocell."""

import sys

import mujoco

model = mujoco.MjModel.from_xml_path("...")
data = mujoco.MjData(model)

sys.stdout.write(f"nq = {model.nq}  nv = {model.nv}  nu = {model.nu}\n")
for index in range(model.njnt):
    sys.stdout.write(f"joint {index} {model.joint(index).name}\n")
for index in range(model.nu):
    sys.stdout.write(
        f"actuator {index} {model.actuator(index).name} "
        f"ctrlrange = {model.actuator_ctrlrange[index]}\n"
    )

sys.stdout.write("Hello world!\n")
