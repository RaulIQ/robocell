"""Typed scalar in a synthetic drive dictionary."""

from dataclasses import dataclass
from typing import final, override

from canopen.objectdictionary import ODVariable

from robocell_sim.hardware.interfaces.definition import Definition


@final
@dataclass(frozen=True)
class Defined(Definition):
    """Definition that binds an object address to its type and initial value."""

    _address: tuple[int, int]
    _description: tuple[int, int, str]

    @override
    def object(self) -> ODVariable:
        variable = ODVariable(f"Object {self._address}", *self._address)
        variable.data_type, variable.default, variable.access_type = (
            self._description
        )
        return variable
