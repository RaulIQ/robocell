"""Report that publishes several reports."""

from dataclasses import dataclass
from typing import final, override

from robocell_sim.hardware.interfaces.report import Report


@final
@dataclass(frozen=True)
class Joined(Report):
    """Report that publishes each of its reports once, in the given order.

    The reports go out one after another on every publication, so a list of
    reports, each with its own period, reads as one schedule from top to
    bottom.
    """

    _reports: tuple[Report, ...]

    @override
    def publish(self) -> None:
        for report in self._reports:
            report.publish()
