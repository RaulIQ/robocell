"""Advance an active motor pilot and publish its ROS feedback."""

from dataclasses import dataclass
from logging import Logger
from typing import final

from std_msgs.msg import String  # type: ignore[import-untyped]  # ROS generates message modules without types

from robocell_sim.robot.interfaces.pilot import Pilot
from robocell_sim.ros.implementations.motor.reported import Reported
from robocell_sim.ros.interfaces.topic import Topic

__all__ = ["Progressed", "String"]


@final
@dataclass(frozen=True)
class Progressed:
    """Publish drive status and deduplicate wheel acquisitions by timestamp."""

    _names: tuple[str, str]
    _topics: tuple[Topic, Topic]
    _log: Logger
    _state: list[tuple[str, int]]

    def __call__(self, pilot: Pilot, moment: float, simulation: int) -> None:
        """Poll the pilot and publish each newly measured sample."""
        condition, measurements = pilot.advance(moment, simulation)
        status, stamp = self._state[0]
        if condition != status:
            self._log.info("CANopen host is %s", condition)
            self._topics[1].publish(String(data=condition))
        for sample in measurements:
            if sample[0] != stamp:
                Reported(sample, self._names, self._topics[0]).publish()
                stamp = sample[0]
        self._state[0] = condition, stamp
