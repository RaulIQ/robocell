"""A stable name for an allocated pseudo-terminal."""

from collections.abc import Generator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import final, override

from robocell_sim.hardware.interfaces.endpoint import Endpoint


@final
@dataclass(frozen=True)
class Named(Endpoint):
    """Endpoint owning an exclusive symlink during its binding."""

    _path: Path
    _target: str

    @override
    @contextmanager
    def binding(self) -> Generator[Path]:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._path.symlink_to(self._target)
        try:
            yield self._path
        finally:
            if self._path.is_symlink() and self._path.readlink() == Path(
                self._target
            ):
                self._path.unlink()
