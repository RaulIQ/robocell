"""An explicit transform history for motion-compensation tests."""

from typing import Protocol

from tf2_ros import Buffer  # type: ignore[import-untyped]  # ROS generates message modules without types


class History(Protocol):
    """TF measurements enclosing the acquisition interval."""

    def buffer(self) -> Buffer:
        """Build the measured TF history."""
        ...
