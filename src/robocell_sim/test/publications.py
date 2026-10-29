"""ROS scans collected from a device conversation."""

from typing import Protocol

from sensor_msgs.msg import LaserScan  # type: ignore[import-untyped]  # ROS generates message modules without types


class Publications(Protocol):
    """Publications that expose the ROS output of a bounded serial scenario."""

    def messages(self) -> tuple[LaserScan, ...]:
        """Run the scenario and collect its scan publications."""
        ...
