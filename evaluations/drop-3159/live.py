# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/drop-3159/live.py

3.159 criterion D7 (`docs/plans/ros-alignment/3.159-drop-detection.md`):
one explore mission on `home_first_floor_stairs` through the brain's HTTP
API, nav2 on, on a fresh `tests/demo_explore.py` `stack()` (which takes the
desktop's exclusive lock). `demo_explore.run_mission` judges contacts; a
thread beside it reads `/world/truth` five times a second and judges the
stairwell on `tests/drop_sweep.py`'s ground truth (falls, overhang, gap).

    PICAR_ROS_NETWORK=host ROS_IMAGE=vision-picar-ros:latest \\
        python evaluations/drop-3159/live.py OUT.json
"""

import json
import math
import os
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.maps import build_world  # noqa: E402
from tests import demo_explore as de  # noqa: E402
from tests.drop_sweep import drop_truth  # noqa: E402

HOUSE = "home_first_floor_stairs"


def main():
    out = Path(sys.argv[1])
    world = build_world(HOUSE)
    truth = {"samples": 0, "fell": False, "overhang_cm": 0.0, "min_gap_cm": math.inf}
    done = threading.Event()
    robot = de.stack(HOUSE)
    # 3.79: a live run at load above ~4 is invalid; read once the exclusive
    # lock is held and the stack is up.
    load = os.getloadavg()

    def watch():
        import httpx
        client = httpx.Client(base_url=f"http://127.0.0.1:{de.ROBOT}", timeout=10)
        while not done.is_set():
            try:
                t = client.get("/world/truth").json()
            except Exception:  # noqa: BLE001 -- a slow reply: next sample
                time.sleep(0.2)
                continue
            world.x, world.y = t["x_m"] / 0.3, t["y_m"] / 0.3
            world.theta = math.radians(t["heading_deg"]) - math.pi / 2
            fell, over, gap = drop_truth(world)
            truth["samples"] += 1
            truth["fell"] = truth["fell"] or fell
            truth["overhang_cm"] = max(truth["overhang_cm"], round(over, 2))
            truth["min_gap_cm"] = min(truth["min_gap_cm"], round(gap, 2))
            time.sleep(0.2)

    watcher = threading.Thread(target=watch, daemon=True)
    watcher.start()
    try:
        result = de.run_mission(robot, HOUSE)
    finally:
        done.set()
        watcher.join(5)
        de._kill_ports()
        os.system(f"docker rm -f {de.CONTAINER} >/dev/null 2>&1")
    row = {"house": HOUSE, "image": os.environ.get("ROS_IMAGE", "vision-picar-ros:latest"),
           "mission": result, "drop_truth": truth, "run_dir": de.RUN_DIR,
           "load_at_start": [round(v, 2) for v in load]}
    out.write_text(json.dumps(row, indent=1) + "\n")
    print(json.dumps(row))


if __name__ == "__main__":
    main()
