"""A bounded simulator run with real physics and queues, without GUI."""

import contextlib
import logging
import multiprocessing
import multiprocessing.context
import multiprocessing.process
import multiprocessing.queues
import os
import runpy
import secrets
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import final, override

import can
import mujoco
import mujoco.viewer
import pytest
import rclpy
from can.interfaces.virtual import VirtualBus
from rclpy.utilities import get_default_context

from test.boot import Boot


@final
@dataclass(frozen=True)
class Booted(Boot):
    """Startup with virtual CAN, intercepted forks and one viewer frame."""

    _flags: tuple[str, ...]
    _directory: Path
    _patch: pytest.MonkeyPatch

    @override
    def events(self) -> tuple[int, list[bool], list[int]]:
        queues: list[multiprocessing.queues.Queue[object]] = []
        starts: list[bool] = []
        self.queues(multiprocessing.get_context("fork"), queues)
        self.processes(starts)
        self.viewer()
        self.environment()
        runpy.run_path(str(Path(__file__).parents[1] / "robocell_sim/main.py"))
        return (len(queues), starts, [queue.qsize() for queue in queues])

    @override
    def queues(
        self,
        context: multiprocessing.context.BaseContext,
        queues: list[multiprocessing.queues.Queue[object]],
    ) -> None:
        """Record allocated queues while keeping their real storage."""
        origin = context.Queue

        def allocate(maxsize: int = 0) -> multiprocessing.queues.Queue[object]:
            """Keep real queues visible to the test without consuming them."""
            queue = origin(maxsize)
            queues.append(queue)
            return queue

        self._patch.setattr(context, "Queue", allocate)

    @override
    def processes(self, starts: list[bool]) -> None:
        """Replace physical CAN and GUI forks at the process boundary."""
        initialize = rclpy.init
        self._patch.setattr(
            multiprocessing.process.BaseProcess,
            "start",
            lambda _process: starts.append(get_default_context().ok()),
        )
        self._patch.setattr(
            can,
            "Bus",
            lambda **_arguments: VirtualBus(channel=secrets.token_hex(8)),
        )
        self._patch.setattr(
            rclpy,
            "init",
            lambda: initialize(
                domain_id=secrets.SystemRandom().randint(0, 101)
            ),
        )

    @override
    def viewer(self) -> None:
        """Supply a real scene with a single frame and no GUI."""
        steps = iter((True, False))
        self._patch.setattr(
            mujoco.viewer,
            "launch_passive",
            lambda model, _data, **_keywords: contextlib.nullcontext(
                SimpleNamespace(
                    user_scn=mujoco.MjvScene(model, 1000),
                    set_texts=lambda *_texts: None,
                    is_running=lambda: next(steps),
                    lock=contextlib.nullcontext,
                    sync=lambda: None,
                )
            ),
        )

    @override
    def environment(self) -> None:
        """Build a temporary ament index pointing at real package resources."""
        index = self._directory / "share/ament_index/resource_index/packages"
        index.mkdir(parents=True)
        (index / "robocell_sim").touch()
        (self._directory / "share/robocell_sim").symlink_to(
            Path(__file__).parents[1], target_is_directory=True
        )
        self._patch.setenv(
            "AMENT_PREFIX_PATH",
            f"{self._directory}:{os.environ.get('AMENT_PREFIX_PATH', '')}",
        )
        self._patch.setattr(logging.root.manager, "disable", logging.CRITICAL)
        self._patch.setattr(
            sys,
            "argv",
            [
                "main.py",
                *self._flags,
                "--ros-args",
                "-p",
                f"lidar_port:={self._directory / 'lidar'}",
            ],
        )
