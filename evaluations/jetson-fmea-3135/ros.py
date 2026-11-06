# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/ros.py

The ROS container (SLAM + nav2) beside a `stack.py` stack, for the rows
whose robot moves on a nav2 goal (amendment 5): started as 3.135.C's soak
starts it (`vision-picar-ros:latest`, host network, the stack's secret and
URLs), then the faults F4 (`docker kill`) and F5 (`docker pause`) act on it
from outside. The stack must be built with `ROS_ENV` in its `extra_env`.

    st = Stack(out, extra_env=ROS_ENV); st.start(); r = Ros(st, out); r.up() ... r.down()
"""

from __future__ import annotations

import json
import os
import subprocess
import time
from pathlib import Path

from stack import StackError, http

IMAGE = os.environ.get("FMEA_ROS_IMAGE", "vision-picar-ros:latest")
CONTAINER = os.environ.get("FMEA_ROS_CONTAINER", "picar-ros-3135")
BRIDGE = "http://127.0.0.1:8090"
ROS_ENV = {"ROBOT_DRIVE": "ros", "WORLD_MODE": "ros"}


def docker(*args: str, timeout: float = 120.0, env=None) -> subprocess.CompletedProcess:
    return subprocess.run(["docker", *args], capture_output=True, text=True, timeout=timeout, env=env)


class Ros:
    def __init__(self, st, out: Path):
        self.st, self.out = st, Path(out)

    def up(self, bridge_s: float = 180.0, slam_s: float = 120.0) -> None:
        if docker("ps", "-aq", "-f", f"name=^{CONTAINER}$").stdout.strip():
            raise StackError(f"a container named {CONTAINER} exists already")
        if _ok(BRIDGE + "/health"):
            raise StackError(":8090 already answers -- someone else's bridge")
        r = docker("run", "-d", "--name", CONTAINER, "--network", "host",
                   "-e", "APP_SHARED_SECRET", "-e", "ROBOT_URL", "-e", "BRAIN_URL",
                   IMAGE, "ros2", "launch", "picar_bringup", "picar.launch.py",
                   env=dict(os.environ, APP_SHARED_SECRET=self.st.secret,
                            ROBOT_URL=self.st.urls["robot"], BRAIN_URL=self.st.urls["brain"]))
        if r.returncode != 0:
            raise StackError(f"docker run: {r.stderr.strip()[:300]}")
        self.wait_ready(bridge_s, slam_s)

    def wait_ready(self, bridge_s: float = 180.0, slam_s: float = 120.0) -> None:
        _wait(lambda: _ok(BRIDGE + "/health"), bridge_s, "the ROS bridge")
        _wait(lambda: '"start_truth": {' in json.dumps(
            http("GET", BRIDGE + "/slam/pose", secret=self.st.secret, timeout=3)[1]),
            slam_s, "SLAM's start pose")
        # 3.105: goals wait for nav2's lifecycle manager; the bridge says so in /health.
        _wait(lambda: (http("GET", BRIDGE + "/health", timeout=3)[1] or {}).get("nav2", {}).get("active")
              is True, 120.0, "nav2 active")

    def kill(self) -> float:
        t = time.monotonic()
        r = docker("kill", CONTAINER, timeout=30)
        if r.returncode != 0:
            raise StackError(f"docker kill: {r.stderr.strip()[:200]}")
        return t

    def pause(self) -> float:
        t = time.monotonic()
        r = docker("pause", CONTAINER, timeout=30)
        if r.returncode != 0:
            raise StackError(f"docker pause: {r.stderr.strip()[:200]}")
        return t

    def unpause(self) -> float:
        t = time.monotonic()
        r = docker("unpause", CONTAINER, timeout=30)
        if r.returncode != 0:
            raise StackError(f"docker unpause: {r.stderr.strip()[:200]}")
        return t

    def restart(self) -> None:
        """A killed container started again (F4's recovery): a new container,
        a fresh SLAM session, as `picar-ros`'s unit would bring it back."""
        self.down(logs_as="ros-before-restart.log")
        self.up()

    def down(self, logs_as: str = "ros.log") -> None:
        try:
            docker("unpause", CONTAINER, timeout=30)          # a paused container cannot be removed cleanly
            with open(self.out / logs_as, "w") as f:
                subprocess.run(["docker", "logs", CONTAINER], stdout=f, stderr=subprocess.STDOUT,
                               timeout=60)
        except (OSError, subprocess.SubprocessError):
            pass
        docker("rm", "-f", CONTAINER, timeout=60)


def goal_state(st) -> str | None:
    """The live nav2 goal's state as the robot server sees it (`active`,
    `pending`, ...), or None with no goal."""
    try:
        code, body = st.robot("GET", "/world/goal", timeout=3)
    except OSError:
        return None
    g = body.get("goal") if code == 200 and isinstance(body, dict) else None
    return g.get("state") if isinstance(g, dict) else None


def _ok(url: str) -> bool:
    try:
        return http("GET", url, timeout=2)[0] == 200
    except OSError:
        return False


def _wait(cond, timeout_s: float, what: str) -> None:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            if cond():
                return
        except (OSError, ValueError, AttributeError):
            pass
        time.sleep(1.0)
    raise StackError(f"{what} not reached in {timeout_s:.0f} s")
