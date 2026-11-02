"""Sparse CANopen record with explicit supported subindices."""

from dataclasses import dataclass
from typing import final, override

from canopen.objectdictionary import UNSIGNED8, ODRecord

from robocell_sim.hardware.implementations.motor.defined import Defined
from robocell_sim.hardware.interfaces.definition import Definition


@final
@dataclass(frozen=True)
class Grouped(Definition):
    """Record whose count describes the highest supported subindex."""

    _index: int
    _entries: tuple[tuple[int, int, int, str], ...]

    @override
    def object(self) -> ODRecord:
        record = ODRecord(f"Record {self._index:04X}", self._index)
        for subindex, datatype, value, access in (
            (0, UNSIGNED8, max(entry[0] for entry in self._entries), "ro"),
            *self._entries,
        ):
            record.add_member(
                Defined(
                    (self._index, subindex), (datatype, value, access)
                ).object()
            )
        return record
