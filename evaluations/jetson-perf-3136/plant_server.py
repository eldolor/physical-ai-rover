"""
evaluations/jetson-perf-3136/plant_server.py

Criterion 3's desktop robot server: the shipped `robot.server` app in sim
mode with two delays that can be planted from outside, through flag files
in $PLANT_DIR (checked on every call, so a check turns them on and off
without a restart):

* `wheel` -- the next simulated `advance()` (called inside the wheel loop's
  tick, on the event loop) blocks 100 ms, once, then removes the flag. The
  next tick's `dt` is the period plus 100 ms: P4's `max_dt_s` must show it.
* `map`   -- every `world.get_map()` (inside `/world/map`) sleeps 100 ms
  while the flag exists: P12's reply time must show it.

    PLANT_DIR=/tmp/x python evaluations/jetson-perf-3136/plant_server.py <port>

Nothing in robot/ or sim/ is edited; the patch is on the classes, here.
"""

import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

from sim.mock_robot import MockRobot  # noqa: E402
from sim.mock_world import MockWorld  # noqa: E402

PLANT = Path(os.environ["PLANT_DIR"])
DELAY_S = float(os.environ.get("PLANT_DELAY_S", "0.1"))
_advance, _get_map = MockRobot.advance, MockWorld.get_map


def advance(self, dt):
    flag = PLANT / "wheel"
    if flag.exists():
        flag.unlink()
        time.sleep(DELAY_S)       # blocks the event loop, as a slow route would
        (PLANT / "wheel.done").write_text(f"{time.time():.3f}\n")
    return _advance(self, dt)


def get_map(self):
    if (PLANT / "map").exists():
        time.sleep(DELAY_S)
    return _get_map(self)


MockRobot.advance, MockWorld.get_map = advance, get_map

import uvicorn  # noqa: E402

from robot.server import create_app  # noqa: E402

uvicorn.run(create_app(os.environ.get("ROBOT_CONFIG_PATH")), host="127.0.0.1",
            port=int(sys.argv[1]), log_level="warning")
