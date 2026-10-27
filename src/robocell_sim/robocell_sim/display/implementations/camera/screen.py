"""Window that shows the robot camera."""

import contextlib
import multiprocessing.queues
import queue
from dataclasses import dataclass
from typing import final, override

import cv2
import mujoco
import numpy as np
import numpy.typing as npt

from robocell_sim.display.interfaces.video import Video
from robocell_sim.hardware.implementations.camera.rendered import Rendered


@final
@dataclass(frozen=True)
class Screen(Video):
    """OpenCV window that renders the robot camera in a process of its own.

    The queue carries joint positions from the simulation to the window
    and drops them while it is full, so a slow window never holds the
    simulation back. The model describes the world the window renders on
    its own copy of the state, the camera names the camera of the model,
    and the size is the width and the height of the image in pixels.
    """

    _queue: multiprocessing.queues.Queue[npt.NDArray[np.float64]]
    _model: mujoco.MjModel
    _camera: str
    _size: tuple[int, int]

    @override
    def record(self, state: npt.NDArray[np.float64]) -> None:
        with contextlib.suppress(queue.Full):
            self._queue.put_nowait(state.copy())

    @override
    def show(self) -> None:
        data = mujoco.MjData(self._model)
        camera = Rendered(
            mujoco.Renderer(self._model, self._size[1], self._size[0]),
            data,
            self._camera,
        )
        cv2.namedWindow("Robot camera")
        while cv2.getWindowProperty("Robot camera", cv2.WND_PROP_VISIBLE) >= 1:
            with contextlib.suppress(queue.Empty):
                data.qpos[:] = self._queue.get(timeout=0.1)
                mujoco.mj_forward(self._model, data)
                cv2.imshow(
                    "Robot camera",
                    cv2.cvtColor(camera.shot(), cv2.COLOR_RGB2BGR),
                )
            cv2.waitKey(1)
