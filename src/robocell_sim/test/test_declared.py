"""Tests for the settings that a ROS 2 node declares as its parameters."""

import secrets

from hamcrest import assert_that, close_to, equal_to
from rclpy.context import Context
from rclpy.node import Node
from robocell_sim.ros.implementations.settings.declared import Declared


def test_declared_falls_back_to_the_default_number() -> None:
    """A number nobody overrides comes back as its default."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    default = secrets.SystemRandom().uniform(-1e3, 1e3)
    assert_that(
        Declared(
            Node(f"declared{secrets.token_hex(4)}", context=context)
        ).number(f"ratio{secrets.token_hex(3)}", default),
        close_to(default, 1e-12),
        "settings did not fall back to the default number",
    )


def test_declared_reads_the_overridden_number() -> None:
    """A number the launch overrides comes back as the launch set it."""
    name = f"ratio{secrets.token_hex(3)}"
    value = secrets.SystemRandom().uniform(-1e3, 1e3)
    context = Context()
    context.init(
        args=["--ros-args", "-p", f"{name}:={value:.6f}"],
        domain_id=secrets.SystemRandom().randint(0, 101),
    )
    assert_that(
        Declared(
            Node(f"declared{secrets.token_hex(4)}", context=context)
        ).number(name, secrets.SystemRandom().uniform(-1e3, 1e3)),
        close_to(value, 1e-6),
        "settings did not read the overridden number",
    )


def test_declared_falls_back_to_the_default_whole() -> None:
    """A whole number nobody overrides comes back as its default."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    default = secrets.SystemRandom().randint(-(2**40), 2**40)
    assert_that(
        Declared(
            Node(f"declared{secrets.token_hex(4)}", context=context)
        ).whole(f"count{secrets.token_hex(3)}", default),
        equal_to(default),
        "settings did not fall back to the default whole number",
    )


def test_declared_reads_the_overridden_whole() -> None:
    """A whole number the launch overrides comes back as the launch set it."""
    name = f"count{secrets.token_hex(3)}"
    value = secrets.SystemRandom().randint(-(2**40), 2**40)
    context = Context()
    context.init(
        args=["--ros-args", "-p", f"{name}:={value}"],
        domain_id=secrets.SystemRandom().randint(0, 101),
    )
    assert_that(
        Declared(
            Node(f"declared{secrets.token_hex(4)}", context=context)
        ).whole(name, secrets.SystemRandom().randint(-(2**40), 2**40)),
        equal_to(value),
        "settings did not read the overridden whole number",
    )


def test_declared_falls_back_to_the_default_text() -> None:
    """A text nobody overrides comes back as its default."""
    context = Context()
    context.init(domain_id=secrets.SystemRandom().randint(0, 101))
    default = f"ünïcode-{secrets.token_hex(9)}"
    assert_that(
        Declared(Node(f"declared{secrets.token_hex(4)}", context=context)).text(
            f"label{secrets.token_hex(3)}", default
        ),
        equal_to(default),
        "settings did not fall back to the default text",
    )


def test_declared_reads_the_overridden_text() -> None:
    """A text the launch overrides comes back as the launch set it."""
    name = f"label{secrets.token_hex(3)}"
    value = f"ïmu-{secrets.token_hex(9)}"
    context = Context()
    context.init(
        args=["--ros-args", "-p", f"{name}:={value}"],
        domain_id=secrets.SystemRandom().randint(0, 101),
    )
    assert_that(
        Declared(Node(f"declared{secrets.token_hex(4)}", context=context)).text(
            name, f"link-{secrets.token_hex(5)}"
        ),
        equal_to(value),
        "settings did not read the overridden text",
    )
