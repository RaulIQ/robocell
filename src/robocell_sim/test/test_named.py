"""Tests for stable serial endpoint ownership."""

from pathlib import Path

import pytest
from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.uart.named import Named


def test_named_keeps_the_endpoint_only_while_bound(tmp_path: Path) -> None:
    """A live simulator owns its name and removes it when its context exits."""
    target = tmp_path / "actual"
    target.touch()
    path = tmp_path / "device"
    with Named(path, str(target)).binding():
        connected = path.resolve() == target
    assert_that(
        (connected, path.is_symlink()),
        equal_to((True, False)),
        "named endpoint did not follow its owner's lifetime",
    )


def test_named_cannot_overwrite_an_existing_endpoint(tmp_path: Path) -> None:
    """A competing process cannot replace an existing endpoint name."""
    path = tmp_path / "device"
    path.write_text("owned")
    with (
        pytest.raises(FileExistsError),
        Named(path, str(tmp_path / "other")).binding(),
    ):
        pass
    assert_that(
        path.read_text(),
        equal_to("owned"),
        "competing endpoint did not preserve the existing owner",
    )
