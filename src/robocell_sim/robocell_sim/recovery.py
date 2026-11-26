"""Evidence used to protect the map when wheel odometry disagrees with depth."""

from collections.abc import Sequence
from math import dist, isfinite, remainder, tau
from statistics import median


def blocked(before: Sequence[float], after: Sequence[float], travel: float) -> bool:
    """Detect wheel travel while overlapping depth returns stay in place."""
    if len(before) != len(after) or travel < 0.1:
        return False
    changes = [
        abs(left - right)
        for left, right in zip(before, after)
        if isfinite(left) and isfinite(right)
    ]
    return len(changes) >= 60 and median(changes) < 0.45 * travel


def confirmed(
    correction: tuple[float, float, float],
    history: Sequence[tuple[float, float, float]],
) -> bool:
    """Require three accepted matches with a consistent map correction."""
    return len(history) == 2 and all(
        dist(correction[:2], previous[:2]) < 0.08
        and abs(remainder(correction[2] - previous[2], tau)) < 0.08
        for previous in history
    )
