"""Immutable control and acquisition session of one CAN drive."""

from robocell_sim.hardware.interfaces.motor import Motor

Session = tuple[Motor, tuple[float, float, int], int]
