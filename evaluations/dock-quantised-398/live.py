# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/dock-quantised-398/live.py -- 3.198's live run (Q7):
`tests/demo_explore.py`'s stack (robot server, brain, the ROS container with
nav2 and slam_toolbox, `drive: ros`), `scaled_house`, finds of the red
backpack through the brain's HTTP API, judged on the simulator's ground truth.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.198 python evaluations/dock-quantised-398/live.py OUT.jsonl [N]

The first find starts where the stack starts (3.195's c-runs: the dock
(1.95, 1.35) facing east); each later one first takes a person's nav2 goal to
one of 3.156.B's spread starts, which is then that mission's dock (D1). Per
mission: 3.156's record (`evaluations/dock-3156/live.py` `mission()`:
outcome, `return`, the error against the brain's dock, contacts) plus the
TRUE error at the end against the TRUE pose at the start, the return's
correcting goals (3.198 D2), and the log's return lines.
"""

import importlib.util
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import httpx  # noqa: E402

from tests import demo_explore as de  # noqa: E402

_spec = importlib.util.spec_from_file_location("live3156", ROOT / "evaluations" / "dock-3156" / "live.py")
L = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(L)

HOUSE = "scaled_house"
LIMIT_S = 300
WALL_S = 700


def main(argv):
    out = Path(argv[0])
    n = int(argv[1]) if len(argv) > 1 else 5
    cells = L.spread_cells(HOUSE, n)
    robot = de.stack(HOUSE)
    try:
        brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
        truth = L.Truth(robot, HOUSE)
        for _ in range(60):
            p = robot.get("/world/pose").json()
            if p.get("usable"):
                break
            time.sleep(2)
        for i in range(n):
            start_goal = None
            if i:
                c = cells[i]
                start_goal = L.goto(robot, (c[0] + 0.5) * 0.3, (c[1] + 0.5) * 0.3)
                time.sleep(3.0)          # the person's authority lapses
            t0 = truth.pose()
            body = {"target_object": de.TARGET, "policy": "explore", "max_steps": 400,
                    "mission_time_limit_s": LIMIT_S}
            rec = L.mission(robot, brain, truth, body, wall_s=WALL_S)
            x, y, h = truth.pose()
            status = brain.get("/mission/status").json()
            rec.update(i=i, house=HOUSE, start_goal=start_goal,
                       truth_start=[round(v, 4) for v in t0],
                       truth_end=[round(x, 4), round(y, 4), round(h, 2)],
                       truth_err_m=round(math.hypot(x - t0[0], y - t0[1]), 4),
                       truth_err_deg=round(abs((h - t0[2] + 180) % 360 - 180), 2),
                       fine_goals=(rec.get("ret") or {}).get("fine_goals"),
                       log_return=[ln for ln in (status.get("log_tail") or []) if "return" in ln][-6:])
            print(json.dumps(rec), flush=True)
            with open(out, "a") as f:
                f.write(json.dumps(rec) + "\n")
    finally:
        import subprocess
        subprocess.run(["docker", "rm", "-f", de.CONTAINER], capture_output=True)
        de._kill_ports()


if __name__ == "__main__":
    main(sys.argv[1:])
