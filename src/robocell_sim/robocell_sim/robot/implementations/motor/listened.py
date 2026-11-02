"""Bounded CAN reception and independent node routing."""

from dataclasses import dataclass
from typing import final, override

import can

from robocell_sim.robot.interfaces.listening import Listening
from robocell_sim.robot.interfaces.pilot import Session

_BUDGET = 256
_SIZE = 8
_PARTS = {0x180: 1, 0x280: 2, 0x380: 3}
_BASES = (*_PARTS, 0x80, 0x580, 0x700)


@final
@dataclass(frozen=True)
class Listened(Listening):
    """Reception that bounds traffic and preserves separate wheel epochs."""

    _bus: can.BusABC
    _numbers: tuple[int, int]
    _session: Session

    @override
    def advance(self, moment: float, clock: int) -> Session:
        session = self._session
        for _ in range(_BUDGET):
            message = self._bus.recv(0)
            if message is None:
                break
            session = Listened(self._bus, self._numbers, session).read(
                message, moment
            )
        else:
            error = "Host CAN service exhausted its 256-frame budget"
            raise BufferError(error)
        return session

    @override
    def read(self, message: can.Message, moment: float) -> Session:
        if message.is_error_frame:
            error = "CAN controller reported a bus error during drive service"
            raise OSError(error)
        axes = list(self._session[0])
        for offset, number in enumerate(self._numbers):
            if (
                not (
                    message.is_extended_id
                    or message.is_remote_frame
                    or message.is_fd
                )
                and message.arbitration_id - number in _BASES
            ):
                self.verify(message, offset)
                axes[offset] = axes[offset].read(
                    message,
                    _PARTS.get(message.arbitration_id - number, 0),
                    moment,
                )
        return (tuple(axes), *self._session[1:])

    @override
    def verify(self, message: can.Message, offset: int) -> None:
        number = self._numbers[offset]
        if (
            self._session[1][0] == "enabled"
            and message.arbitration_id == 0x700 + number
            and bytes(message.data) == b"\0"
        ):
            error = (
                f"Drive {number} rebooted and invalidated its feedback epoch"
            )
            raise OSError(error)
        if (
            self._session[1][0] == "enabled"
            and message.arbitration_id == 0x80 + number
            and len(message.data) == _SIZE
            and int.from_bytes(message.data[:2], "little")
        ):
            error = (
                f"Drive {number} reported emergency "
                f"{bytes(message.data[:2]).hex()}"
            )
            raise OSError(error)
