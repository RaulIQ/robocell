"""Publication of coherent IMU observations received over CAN."""

from dataclasses import dataclass
from typing import final, override

from rclpy.node import Node

from robocell_sim.hardware.interfaces.report import Report
from robocell_sim.robot.implementations.can.drained import Drained
from robocell_sim.robot.implementations.imu.logged import Logged
from robocell_sim.robot.implementations.imu.received import Received as Packet
from robocell_sim.robot.implementations.imu.scaled import Scaled
from robocell_sim.robot.interfaces.pairing import Pairing
from robocell_sim.ros.implementations.imu.inertial import Inertial
from robocell_sim.ros.interfaces.topic import Topic
from robocell_sim.ros.messages import Header


@final
@dataclass(frozen=True)
class Received(Report):
    """Pair CAN frames and stamp each complete IMU observation for ROS."""

    _inbox: Drained
    _pairing: list[Pairing]
    _frame: tuple[Node, str]
    _output: tuple[Logged, Topic, tuple[float, float]]

    @override
    def publish(self) -> None:
        journal, topic, variances = self._output
        node, frame = self._frame
        for identifier, payload in self._inbox.frames():
            pairing = self._pairing[0].read(Packet(identifier, payload))
            self._pairing[0] = pairing
            pairing.note(journal)
            if pairing.complete():
                Inertial(
                    Scaled(pairing),
                    Header(
                        stamp=node.get_clock().now().to_msg(), frame_id=frame
                    ),
                    variances,
                    topic,
                ).publish()
