"""Graph of a node once discovery has caught up with one topic."""

import time
from dataclasses import dataclass
from typing import final, override

from rclpy.node import Node
from rclpy.topic_endpoint_info import TopicEndpointInfo

from test.graph import Graph


@final
@dataclass(frozen=True)
class Settled(Graph):
    """Graph that waits for a topic before it answers.

    The node asks the graph, the topic is the one the test expects to find,
    and the timeout bounds the wait in seconds. A new endpoint reaches the
    graph cache only once DDS discovery sees it, on a thread of its own, so
    each read polls its own query until the topic shows up or the time runs
    out, and then answers with what it last saw, so a real regression still
    fails, only later.
    """

    _node: Node
    _topic: str
    _timeout: float

    @override
    def publishers(self) -> dict[str, list[str]]:
        deadline = time.monotonic() + self._timeout
        while (
            self._topic
            not in (
                graph := dict(
                    self._node.get_publisher_names_and_types_by_node(
                        self._node.get_name(), "/"
                    )
                )
            )
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        return graph

    @override
    def subscribers(self) -> dict[str, list[str]]:
        deadline = time.monotonic() + self._timeout
        while (
            self._topic
            not in (
                graph := dict(
                    self._node.get_subscriber_names_and_types_by_node(
                        self._node.get_name(), "/"
                    )
                )
            )
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        return graph

    @override
    def publications(self) -> tuple[TopicEndpointInfo, ...]:
        deadline = time.monotonic() + self._timeout
        while (
            not (
                infos := tuple(
                    self._node.get_publishers_info_by_topic(self._topic)
                )
            )
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        return infos
