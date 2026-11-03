# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/stack.py

The stack a fault is injected into, started as the board's own run.sh starts
it (sim mode, `SIM_MOTOR_BOARD=fake`, `SIM_LIDAR=fake`), on this section's
ports (`plan_section.py env`: 10350-10354 by default), on whatever machine
runs this file: the desktop for criterion 4, the board (in `~/bench-3135`,
over ssh) for the board runs. Every process is its own process group, so an
injector signals exactly one role and the teardown leaves nothing behind.

Roles: `body` (truth_body.py: physics, the fake board, the truth recorder),
`sensors` (two sensor programs), `lidar` (sim/fake_lidar.py on its own pty
link), `robot` (the robot server; robot_plant.py only when a plant is
named), `brain` (control/brain_server.py). The environment is built from
nothing but what the stack needs: no secret, URL or bucket of the shell's
leaks in. The cloud is never reached: `VISION_URL` points at a closed local
port unless a row's own stub is given (3.135: no spending), metrics and the
inventory stay local, under the run's directory.
"""

from __future__ import annotations

import json
import os
import secrets
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PY = sys.executable

PORTS = {"robot": int(os.environ.get("PICAR_ROBOT_PORT", 10350)),
         "brain": int(os.environ.get("PICAR_BRAIN_PORT", 10351)),
         "body": int(os.environ.get("PICAR_BODY_PORT", 10352)),
         "sensor": int(os.environ.get("PICAR_SENSOR_PORT", 10353)),
         "sensor2": int(os.environ.get("PICAR_SENSOR2_PORT", 10354))}
CLOSED_PORT = 9          # discard: nothing listens; a refused connection, never a paid call
ROBOT_PLANTS = {"watchdog", "lidar_fresh", "feedback_fresh", "stop_waits", "wall_clock", "no_latch"}
BODY_PLANTS = {"heartbeat"}
BRAIN_PLANTS = {"cloud_class", "cloud_deadline", "vlm_absent", "write_intolerant"}
KEEP_ENV = ("PATH", "HOME", "USER", "LANG", "LC_ALL", "TZ", "TMPDIR", "XDG_RUNTIME_DIR")


class StackError(RuntimeError):
    pass


def http(method: str, url: str, body: Optional[dict] = None, secret: str = "",
         timeout: float = 5.0) -> tuple:
    """(status, json-or-text). Never raises on an HTTP status."""
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(url, data=data, method=method,
                                 headers={"content-type": "application/json",
                                          **({"x-app-secret": secret} if secret else {})})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode()
            code = r.status
    except urllib.error.HTTPError as e:
        raw, code = e.read().decode(), e.code
    try:
        return code, json.loads(raw)
    except ValueError:
        return code, raw


class Stack:
    def __init__(self, out: Path, *, house: str = "scaled_house", plant: str = "",
                 vision_url: Optional[str] = None, extra_env: Optional[dict] = None):
        plants = {p for p in plant.split(",") if p}
        if plants - ROBOT_PLANTS - BODY_PLANTS - BRAIN_PLANTS:
            raise StackError(f"unknown plant(s) {sorted(plants - ROBOT_PLANTS - BODY_PLANTS - BRAIN_PLANTS)}")
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.plants = plants
        self.secret = secrets.token_hex(16)
        self.house = house
        self.procs: dict = {}
        self.logs: list = []
        self.urls = {k: f"http://127.0.0.1:{p}" for k, p in PORTS.items()}
        self.shm = f"picar_fmea_{os.getpid()}"
        self.lidar_link = str(self.out / "lidar.pty")
        env = {k: os.environ[k] for k in KEEP_ENV if k in os.environ}
        env.update({
            "PYTHONUNBUFFERED": "1", "APP_SHARED_SECRET": self.secret,
            "ROBOT_SHARED_SECRET": self.secret, "VISION_SHARED_SECRET": "",
            "ROBOT_MODE": "hardware", "SIM_MOTOR_BOARD": "fake", "WORLD_MODE": "none",
            "SIM_MAP": house, "SIM_BODY_SHM": self.shm,
            "SIM_BODY_URL": self.urls["body"],
            "SIM_SENSORS_URL": f"{self.urls['sensor']},{self.urls['sensor2']}",
            "SIM_LIDAR": "fake", "ROBOT_LIDAR": self.lidar_link,
            "ROBOT_URL": self.urls["robot"],
            "VISION_URL": vision_url or f"http://127.0.0.1:{CLOSED_PORT}",
            "INVENTORY_BUCKET": "", "INVENTORY_DIR": str(self.out / "inventory"),
            "INTERVENTION_DIR": str(self.out / "interventions"),
            "PICAR_LOG_LEVEL": "WARNING", "FMEA_DIR": str(self.out),
            "FMEA_PLANT": ",".join(sorted(plants)),
        })
        env.update(extra_env or {})
        self.env = env

    # ------------------------------------------------------------ processes

    def _spawn(self, role: str, argv: list, env: Optional[dict] = None) -> subprocess.Popen:
        log = open(self.out / f"{role}.log", "ab")
        self.logs.append(log)
        p = subprocess.Popen(argv, cwd=ROOT, env=env or self.env, stdout=log,
                             stderr=subprocess.STDOUT, start_new_session=True)
        self.procs[role] = p
        return p

    def _wait_health(self, role: str, url: str, timeout_s: float = 60.0) -> None:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            p = self.procs[role]
            if p.poll() is not None:
                raise StackError(f"{role} exited ({p.returncode}); see {self.out}/{role}.log")
            try:
                if http("GET", url + "/health", timeout=1.0)[0] == 200:
                    return
            except OSError:
                pass
            time.sleep(0.2)
        raise StackError(f"{role} not healthy at {url} in {timeout_s:.0f} s")

    def _uvicorn(self, app: str, port: int) -> list:
        return [PY, "-m", "uvicorn", app, "--host", "127.0.0.1", "--port", str(port),
                "--log-level", "warning"]

    def start(self) -> "Stack":
        busy = [r for r, p in PORTS.items() if _listening(p)]
        if busy:
            raise StackError(f"ports already served: {busy} -- another run on this section's ports?")
        try:
            self._spawn("body", [PY, str(HERE / "truth_body.py"), str(PORTS["body"])],
                        dict(self.env, FMEA_PLANT=",".join(sorted(self.plants & BODY_PLANTS))))
            self._wait_health("body", self.urls["body"])
            self._spawn("sensors", self._uvicorn("sim.sensor_server:app", PORTS["sensor"]))
            self._spawn("sensors2", self._uvicorn("sim.sensor_server:app", PORTS["sensor2"]))
            self._wait_health("sensors", self.urls["sensor"])
            self._wait_health("sensors2", self.urls["sensor2"])
            self._spawn("lidar", [PY, "-m", "sim.fake_lidar", "--link", self.lidar_link])
            end = time.monotonic() + 10
            while not os.path.exists(self.lidar_link) and time.monotonic() < end:
                time.sleep(0.1)
            if not os.path.exists(self.lidar_link):
                raise StackError("the fake lidar made no pty link in 10 s")
            self.start_robot()
            self.start_brain()
        except BaseException:
            self.stop()
            raise
        return self

    def start_robot(self) -> None:
        if self.plants & ROBOT_PLANTS:
            argv = [PY, str(HERE / "robot_plant.py"), str(PORTS["robot"])]
        else:
            argv = self._uvicorn("robot.server:app", PORTS["robot"])     # the shipped code
        self._spawn("robot", argv, dict(self.env, FMEA_PLANT=",".join(sorted(self.plants & ROBOT_PLANTS))))
        self._wait_health("robot", self.urls["robot"])

    def start_brain(self) -> None:
        if self.plants & BRAIN_PLANTS:
            argv = [PY, str(HERE / "brain_plant.py"), str(PORTS["brain"])]
        else:
            argv = self._uvicorn("control.brain_server:app", PORTS["brain"])     # the shipped code
        self._spawn("brain", argv, dict(self.env, FMEA_PLANT=",".join(sorted(self.plants & BRAIN_PLANTS))))
        self._wait_health("brain", self.urls["brain"])

    def pid(self, role: str) -> int:
        return self.procs[role].pid

    def signal(self, role: str, sig: int) -> float:
        """Signal one role's whole process group; returns the monotonic stamp."""
        t = time.monotonic()
        os.killpg(self.procs[role].pid, sig)
        return t

    def alive(self, role: str) -> bool:
        p = self.procs.get(role)
        return p is not None and p.poll() is None

    def stop(self) -> None:
        for role in reversed(list(self.procs)):
            p = self.procs[role]
            if p.poll() is not None:
                continue
            try:
                os.killpg(p.pid, signal.SIGCONT)           # a stopped process cannot exit
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=8)
            except (ProcessLookupError, PermissionError):
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                    p.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        for log in self.logs:
            log.close()
        self.logs = []
        for path in (f"/dev/shm/{self.shm}", self.lidar_link):     # the pty link names a dead pty
            try:
                os.unlink(path)
            except OSError:
                pass

    def __enter__(self) -> "Stack":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # --------------------------------------------------------------- calls

    def robot(self, method: str, path: str, body: Optional[dict] = None, driver: str = "",
              timeout: float = 5.0) -> tuple:
        if driver:
            data = None if body is None else json.dumps(body).encode()
            req = urllib.request.Request(self.urls["robot"] + path, data=data, method=method,
                                         headers={"content-type": "application/json",
                                                  "x-app-secret": self.secret, "x-driver": driver})
            try:
                with urllib.request.urlopen(req, timeout=timeout) as r:
                    return r.status, json.loads(r.read().decode() or "null")
            except urllib.error.HTTPError as e:
                return e.code, e.read().decode()
        return http(method, self.urls["robot"] + path, body, self.secret, timeout)

    def brain(self, method: str, path: str, body: Optional[dict] = None,
              timeout: float = 10.0) -> tuple:
        return http(method, self.urls["brain"] + path, body, self.secret, timeout)

    def now(self) -> dict:
        return http("GET", self.urls["body"] + "/fmea/now", timeout=2.0)[1]


def _listening(port: int) -> bool:
    import socket
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex(("127.0.0.1", port)) == 0
