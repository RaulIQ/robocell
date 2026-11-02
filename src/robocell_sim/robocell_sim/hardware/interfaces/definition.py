"""One typed CANopen object dictionary entry."""

from typing import Protocol

from canopen.objectdictionary import ODRecord, ODVariable


class Definition(Protocol):
    """Definition that creates an independently owned object or record."""

    def object(self) -> ODVariable | ODRecord:
        """Construct the described entry."""
        ...
