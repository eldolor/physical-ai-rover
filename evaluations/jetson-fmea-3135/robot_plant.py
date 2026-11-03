# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/robot_plant.py

Criterion 4's robot server: the shipped `robot.server` app with ONE guard
switched off, named by `FMEA_PLANT` (comma-separated). A run with no plant
never uses this file: `stack.py` starts `uvicorn robot.server:app` itself,
so every scored run is the shipped code. Nothing in robot/ is edited; the
patch is on the module, here.

| Plant | The guard switched off | Row |
|---|---|---|
| `watchdog` | B3.1: `watchdog_should_stop()` never says stop | F6, F7, F13 |
| `lidar_fresh` | 3.42: the LD19 driver calls its scan fresh once any packet came | F9 |
| `feedback_fresh` | 3.34: the board's feedback is called fresh once any frame came | F10 |
| `wall_clock` | the robot server's own timers (`robot.server`'s `time.monotonic`) read the wall clock, plus `FMEA_CLOCK_OFFSET_FILE`'s seconds (the desktop's stand-in for a step) | F16 |
| `stop_waits` | SPEC-REVIEW finding 1 (2026-10-02) undone: under `drive: ros` a stop zeroes the ROS inputs first, synchronously, at the client's 2 s timeout, then the robot | F5 |

    FMEA_PLANT=watchdog python evaluations/jetson-fmea-3135/robot_plant.py <port>
"""

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

PLANTS = {p for p in os.environ.get("FMEA_PLANT", "").split(",") if p}
KNOWN = {"watchdog", "lidar_fresh", "feedback_fresh", "stop_waits", "wall_clock"}
if not PLANTS or PLANTS - KNOWN:
    raise SystemExit(f"FMEA_PLANT must name one or more of {sorted(KNOWN)}, not {sorted(PLANTS)}")

# Importing robot.server builds its `app` (and opens the board's pty) once;
# the patches below are on a module global and on classes, read at each call,
# so they reach that app. A second create_app() would open the pty twice.
import robot.server as server  # noqa: E402

if "watchdog" in PLANTS:
    # Looked up as a module global at each poll, so this reaches the loop.
    server.watchdog_should_stop = lambda last, now, timeout: False
if "lidar_fresh" in PLANTS:
    from robot import lidar_ld19

    lidar_ld19.Ld19Lidar._fresh = lambda self, now: (self._last_packet_at is not None
                                                     and self._scan_newest is not None)
if "feedback_fresh" in PLANTS:
    from robot import hardware_robot

    hardware_robot.HardwareRobot._fresh = lambda self: self._last_frame_at is not None

if "stop_waits" in PLANTS:
    from robot import ros_drive

    def _stop_waits(self):
        with self._gen_lock:
            self._stops += 1
            self._generation += 1
            self._hold_gen = self._generation
            self._hold_until = ros_drive.time.monotonic() + ros_drive.STOP_HOLD_S
        for driver in ("twin-dpad", "brain", "ros"):       # planted: waits on the bridge first
            try:
                self._send(0.0, 0.0, driver=driver, timeout=2.0)
            except Exception:  # noqa: BLE001
                pass
        return self.inner.stop()

    ros_drive.RosDriveRobot.stop = _stop_waits

if "wall_clock" in PLANTS:
    import time as _time
    import types

    _off = {"file": os.environ.get("FMEA_CLOCK_OFFSET_FILE"), "s": 0.0, "read_at": 0.0}

    def _wall() -> float:
        now = _time.time()
        if _off["file"] and now - _off["read_at"] > 0.1:
            try:
                _off["s"] = float(Path(_off["file"]).read_text() or 0)
            except (OSError, ValueError):
                pass
            _off["read_at"] = now
        return now + _off["s"]

    shim = types.ModuleType("time")
    shim.__dict__.update({k: v for k, v in vars(_time).items() if not k.startswith("__")})
    shim.monotonic = _wall                 # planted: robot.server's timers on the wall clock
    server.time = shim

import uvicorn  # noqa: E402

uvicorn.run(server.app, host="127.0.0.1", port=int(sys.argv[1]), log_level="warning")
