"""Host actions played against a device on the simulation thread."""

from dataclasses import dataclass
from typing import final, override

import mujoco
from robocell_sim.hardware.interfaces.port import Port
from robocell_sim.hardware.interfaces.report import Report

from test.awaited import Awaited
from test.conversation import Conversation


@final
@dataclass(frozen=True)
class Played(Conversation):
    """Conversation with timed requests, robot positions and bounded reads."""

    _data: mujoco.MjData
    _report: Report
    _port: Port
    _script: tuple[tuple[tuple[float, bytes, float], ...], float]

    @override
    def responses(self) -> tuple[bytes, ...]:
        replies: list[bytes] = []
        for moment, payload, position in self._script[0]:
            self._data.time = moment
            self._data.joint("base").qpos[0] = position
            mujoco.mj_forward(self._data.model, self._data)
            self._port.send(payload)
            replies.append(
                Awaited(self._report, self._port, self._script[1]).receive()
            )
        return tuple(replies)
