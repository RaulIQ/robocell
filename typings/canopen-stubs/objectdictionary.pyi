UNSIGNED8: int
UNSIGNED16: int
UNSIGNED32: int
INTEGER8: int
INTEGER32: int

class ODVariable:
    data_type: int
    default: int
    access_type: str
    def __init__(self, name: str, index: int, subindex: int = 0) -> None: ...

class ODRecord:
    def __init__(self, name: str, index: int) -> None: ...
    def add_member(self, member: ODVariable) -> None: ...

class ObjectDictionary:
    def __init__(self) -> None: ...
    def __contains__(self, index: int) -> bool: ...
    def add_object(self, obj: ODVariable | ODRecord) -> None: ...
