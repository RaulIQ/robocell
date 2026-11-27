"""Freshness of timestamped target observations."""


def fresh(stamp: float, now: float) -> bool:
    """Check that the observation is at most half a second old."""
    limit = 0.5
    return 0 <= now - stamp <= limit
