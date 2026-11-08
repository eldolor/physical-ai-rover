# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/resume-3156/live.py -- 3.156.D's live run (E1 live, E7):
`tests/demo_explore.py`'s stack on `scaled_house`, the robot started on the
pad, a pack draining fast enough to reach the reserve in a minute or two.

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    ROS_IMAGE=vision-picar-ros:plan-3.156 python evaluations/resume-3156/live.py

Six missions, each from the dock: `full` x 2 (charges, resumes, ends and
docks), `stop` x 2 and `dpad` x 2 (once `status.charge.state` is `charging`,
a person's `POST /stop`, or a D-pad FORWARD). Between missions the idle
robot sits on the pad and charges.
"""

import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

os.environ.update(SIM_BATTERY_V="12.0", SIM_BATTERY_DRAIN_V_PER_S="0.012",
                  SIM_BATTERY_CHARGE_V_PER_S="0.05", RECHARGE_RESUME="1")

import httpx  # noqa: E402

from tests import demo_explore as de  # noqa: E402

HOUSE = "scaled_house"
LIMIT_S = 400
WALL_S = 1200
KINDS = ("full", "full", "stop", "stop", "dpad", "dpad")


def truth(robot):
    t = robot.get("/world/truth").json()
    return t["x_m"], t["y_m"]


def mission(robot, brain, kind):
    r = brain.post("/mission/start", json={"job": "survey", "policy": "explore", "max_steps": 400,
                                           "mission_time_limit_s": LIMIT_S})
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    t0, rec, s = time.time(), {"kind": kind}, {}
    while time.time() - t0 < WALL_S:
        s = brain.get("/mission/status").json()
        charge = s.get("charge") or {}
        if kind != "full" and "drill_at" not in rec and charge.get("state") == "charging":
            rec["drill_at"] = round(time.time() - t0, 1)
            p0 = truth(robot)
            if kind == "stop":
                robot.post("/stop")
                t1, moved = time.time(), 0.0
                while time.time() - t1 < 10.0:
                    p = truth(robot)
                    moved = max(moved, math.hypot(p[0] - p0[0], p[1] - p0[1]))
                    time.sleep(0.2)
                rec["moved_after_stop_cm"] = round(moved * 100, 1)
            else:
                robot.post("/action", json={"action": "FORWARD", "speed": 30, "duration": 0.3},
                           headers={"x-driver": "twin-dpad"})
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    else:
        brain.post("/mission/stop")
        rec["wall_capped"] = True
    rec.update(outcome=s.get("outcome"), end_reason=(s.get("end_reason") or "")[:160],
               ret=s.get("return"), charge=s.get("charge"), error=s.get("error"),
               seconds=round(time.time() - t0), budget=s.get("budget"))
    return rec


def main(argv):
    out = Path(argv[0]) if argv else Path(__file__).with_name("live.jsonl")
    robot = de.stack(HOUSE)
    try:
        brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
        for i, kind in enumerate(KINDS):
            time.sleep(40.0)                # the pad recharges the idle pack
            rec = {"i": i, **mission(robot, brain, kind)}
            print(json.dumps(rec), flush=True)
            with open(out, "a") as f:
                f.write(json.dumps(rec) + "\n")
    finally:
        de._kill_ports()
        os.system(f"docker rm -f {de.CONTAINER} >/dev/null 2>&1")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
