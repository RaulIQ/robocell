"""Target observations and diagnostic state published over ROS."""

from collections.abc import Callable
from dataclasses import dataclass
from typing import final, override

from std_msgs.msg import ColorRGBA, String  # type: ignore[import-untyped]  # ROS generates message modules without types
from visualization_msgs.msg import Marker  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.interfaces.clock import Clock
from robocell_sim.ros.interfaces.sighting import Sighting
from robocell_sim.ros.interfaces.topic import Topic
from robocell_sim.ros.messages import PoseStamped, TransformException, Vector3

__all__ = ["Marker", "Sighted", "String"]

_TIMEOUT = 500_000_000


@final
@dataclass(frozen=True)
class Sighted(Sighting):
    """Publish target poses and retain the last valid acquisition state."""

    _clock: Clock
    _topics: tuple[Topic, Topic, Topic]
    _log: tuple[int, Callable[[str], None], Callable[[str], None]]
    _state: list[tuple[str, int]]

    @override
    def report(self, current: str) -> None:
        """Publish and log each target-state transition once."""
        previous, stamp = self._state[0]
        if previous == current:
            return
        self._state[0] = current, stamp
        self._topics[0].publish(String(data=current))
        self._log[1](f"Target {self._log[0]} {current}")
        if current != "detected":
            self._topics[1].publish(
                Marker(ns="camera_target", action=Marker.DELETE)
            )

    @override
    def unavailable(self, error: TransformException) -> None:
        """Report a real TF failure with its diagnostic detail."""
        if self._state[0][0] != "tf_unavailable":
            self._log[2](f"Target {self._log[0]} TF unavailable: {error}")
        self.report("tf_unavailable")

    @override
    def expire(self) -> None:
        """Mark the target stale when transformed observations stop arriving."""
        current, stamp = self._state[0]
        elapsed = self._clock.now() - stamp
        if current in {"detected", "tf_unavailable"} and elapsed > _TIMEOUT:
            self.report("stale")

    @override
    def publish(self, target: PoseStamped) -> None:
        """Publish the target pose, diagnostic marker and detection state."""
        self._topics[2].publish(target)
        self._state[0] = self._state[0][0], self._clock.now()
        self._topics[1].publish(
            Marker(
                header=target.header,
                ns="camera_target",
                type=Marker.SPHERE,
                pose=target.pose,
                scale=Vector3(x=0.12, y=0.12, z=0.12),
                color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
            )
        )
        self.report("detected")
