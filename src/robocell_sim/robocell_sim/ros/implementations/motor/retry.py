"""Retry services for a verified CANopen enable or explicit fault reset."""

from dataclasses import dataclass
from queue import SimpleQueue
from typing import final

from std_srvs.srv import Trigger  # type: ignore[import-untyped]  # ROS generates message modules without types

__all__ = ["Retry", "Trigger"]


@final
@dataclass(frozen=True)
class Retry:
    """Queue a drive retry and reply with the requested permissions."""

    _requests: SimpleQueue[bool]
    _recovery: bool

    def __call__(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Accept a retry, with fault reset only for the recovery service."""
        self._requests.put(item=self._recovery)
        response.success = True
        response.message = (
            "CANopen recovery requested with fault-reset permission"
            if self._recovery
            else "CANopen retry requested without fault reset"
        )
        return response
