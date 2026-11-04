"""Tests for the recorded occupancy map shipped with the workshop."""

from pathlib import Path

import numpy as np
import yaml
from hamcrest import assert_that, equal_to
from PIL import Image


def test_workshop_map_contains_occupied_free_and_unknown_cells() -> None:
    """The saved map is an occupancy image rather than a serialized graph."""
    directory = Path(__file__).parents[1] / "maps"
    config = yaml.safe_load((directory / "workshop.yaml").read_text())
    with Image.open(directory / config["image"]) as image:
        pixels = set(np.array(image).ravel())
    assert_that(
        pixels,
        equal_to({0, 205, 254}),
        "workshop map did not contain occupied, unknown and free cells",
    )


def test_workshop_map_preserves_mapping_resolution_and_thresholds() -> None:
    """Nav2 can interpret the saved trinary map at the mapping resolution."""
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "maps/workshop.yaml").read_text()
    )
    assert_that(
        {
            name: config[name]
            for name in (
                "resolution",
                "mode",
                "negate",
                "occupied_thresh",
                "free_thresh",
            )
        },
        equal_to(
            {
                "resolution": 0.05,
                "mode": "trinary",
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }
        ),
        "workshop map did not retain the mapping resolution and thresholds",
    )
