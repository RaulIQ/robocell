"""Freshness of timestamped target observations."""


def fresh(stamp: float, now: float) -> bool:
    """Accept observations no older than half a second and never from the future."""
    return 0 <= now - stamp <= 0.5
