"""Verified startup plan for the synthetic wheel profile."""

from typing import Protocol

from robocell_sim.robot.interfaces.negotiation import Negotiation


class Plan(Protocol):
    """Independent startup of one wheel with explicit fault permission."""

    def negotiation(self, *, recovery: bool) -> Negotiation:
        """Build a new bounded zero-motion initialization sequence."""
        ...
