"""Overlay of the viewer."""

from typing import Protocol

import mujoco


class Overlay(Protocol):
    """Overlay that adds geoms of its own on top of the model in a scene."""

    def paint(self, scene: mujoco.MjvScene) -> None:
        """Append the geoms to the scene while it has room for them."""
        ...
