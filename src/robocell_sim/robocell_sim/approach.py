"""A following endpoint separated from the tracked object's occupied cell."""

from math import dist, isfinite


def approach(
    target: tuple[float, float],
    robot: tuple[float, float],
    distance: float,
) -> tuple[float, float]:
    """Choose a point toward the target and stop at the following distance."""
    if (
        not all(isfinite(value) for value in (*target, *robot, distance))
        or distance <= 0
    ):
        error = "Approach requires finite coordinates and a positive distance"
        raise ValueError(error)
    if dist(target, robot) <= distance:
        return robot
    return (
        target[0] + (robot[0] - target[0]) * distance / dist(target, robot),
        target[1] + (robot[1] - target[1]) * distance / dist(target, robot),
    )
