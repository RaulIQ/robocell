"""Bounded drive reconnection initiated by the ROS retry services."""

from contextlib import ExitStack
from dataclasses import dataclass
from logging import Logger
from queue import SimpleQueue
from typing import final, override

import can

from robocell_sim.robot.implementations.motor.hosted import Hosted
from robocell_sim.robot.interfaces.pilot import Pilot
from robocell_sim.ros.interfaces.reconnection import Reconnection

Attempt = tuple[float, float, bool]
_BACKOFF = 0.1


@final
@dataclass(frozen=True)
class Reconnected(Reconnection):
    """Replace the active pilot only on request and retry within a deadline."""

    _connection: tuple[Hosted, ExitStack]
    _requests: SimpleQueue[bool]
    _attempt: list[Attempt]
    _log: tuple[Logger, str, tuple[int, int]]

    def __call__(self, moment: float, pilots: list[Pilot]) -> bool:
        """Apply pending requests and indicate whether the pilot was reset."""
        change = False
        if not self._requests.empty():
            self._connection[1].close()
            pilots.clear()
            self._attempt[:] = [
                (moment + 5.0, -1.0, self._requests.get_nowait())
            ]
            change = True
        return change

    @override
    def connect(self, moment: float, pilots: list[Pilot]) -> None:
        if not self._attempt or pilots:
            return
        deadline, previous, recovery = self._attempt[0]
        if moment < deadline and (
            previous < 0 or moment - previous >= _BACKOFF
        ):
            self._attempt[0] = deadline, moment, recovery
            self.open(pilots, deadline, recovery=recovery)

    @override
    def open(
        self, pilots: list[Pilot], deadline: float, *, recovery: bool
    ) -> None:
        hosted, resources = self._connection
        log, channel, nodes = self._log
        try:
            pilots.append(
                resources.enter_context(
                    hosted.connection(deadline - 5.0, recovery=recovery)
                )
            )
            log.info(
                "CANopen host on %s is verifying drives %d and %d",
                channel,
                *nodes,
            )
        except (can.CanError, OSError) as failure:
            log.warning(
                "CANopen connection on %s failed because %s", channel, failure
            )

    @override
    def expired(self, moment: float) -> bool:
        return bool(self._attempt and moment >= self._attempt[0][0])
