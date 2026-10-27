"""Graph of ROS 2 endpoints."""

from typing import Protocol

from rclpy.topic_endpoint_info import TopicEndpointInfo


class Graph(Protocol):
    """Graph of ROS 2 endpoints as one node sees it."""

    def publishers(self) -> dict[str, list[str]]:
        """Return the types of every topic the node publishes, by name."""
        ...

    def subscribers(self) -> dict[str, list[str]]:
        """Return the types of every topic the node subscribes to, by name."""
        ...

    def publications(self) -> tuple[TopicEndpointInfo, ...]:
        """Return every publisher of the topic with its QoS."""
        ...
