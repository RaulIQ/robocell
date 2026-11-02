"""Tests for the synthetic drive object dictionary."""

from hamcrest import assert_that, equal_to
from robocell_sim.hardware.implementations.motor.described import Described


def test_described_maps_the_controlword_before_the_signed_velocity() -> None:
    """The receive mapping must match the reference six-byte command."""
    node = Described(37).node()
    assert_that(
        tuple(
            int.from_bytes(node.get_data(0x1600, sub), "little")
            for sub in (1, 2)
        ),
        equal_to((0x60400010, 0x60FF0020)),
        "dictionary did not describe the command PDO",
    )


def test_described_uses_the_configured_node_in_its_feedback_identifier() -> (
    None
):
    """Each emulator has distinct feedback identifiers."""
    assert_that(
        int.from_bytes(Described(53).node().get_data(0x1800, 1), "little"),
        equal_to(0x180 + 53),
        "dictionary did not use its node address",
    )
