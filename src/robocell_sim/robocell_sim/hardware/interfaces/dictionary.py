"""Description of a synthetic drive's CANopen objects."""

from typing import Protocol

import canopen


class Dictionary(Protocol):
    """Dictionary that constructs one independently owned local node."""

    def node(self) -> canopen.LocalNode:
        """Create the node and its volatile object dictionary."""
        ...
