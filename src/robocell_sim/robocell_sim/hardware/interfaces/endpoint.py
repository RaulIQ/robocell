"""A named endpoint for a device connection."""

from contextlib import AbstractContextManager
from pathlib import Path
from typing import Protocol


class Endpoint(Protocol):
    """Endpoint whose public path exists only while its owner is running."""

    def binding(self) -> AbstractContextManager[Path]:
        """Bind the public path for the lifetime of the context."""
        ...
