"""ROS service for pausing and resuming the physics clock."""

from dataclasses import dataclass
from threading import Event
from typing import final

from std_srvs.srv import SetBool  # type: ignore[import-untyped]  # ROS generates service modules without types

__all__ = ["Frozen", "SetBool"]


@final
@dataclass(frozen=True)
class Frozen:
    """Set the shared physics pause flag on an explicit service request."""

    _pause: Event

    def __call__(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Reply with the requested physics state."""
        self._pause.set() if request.data else self._pause.clear()
        response.success = True
        response.message = (
            "Simulation paused" if request.data else "Simulation resumed"
        )
        return response
