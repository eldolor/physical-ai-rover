# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/dock-3156/live.py -- 3.156.B's live run (C1 live, C2, C3, C7):
`tests/demo_explore.py`'s stack (robot server, brain, the ROS container with
nav2 and slam_toolbox), missions through the brain's HTTP API, judged on the
simulator's ground truth (`/world/truth`, `/sim/objects`, the chassis
footprint of `tests/footprint_sweep.py`).

    eval "$(python tools/plan_section.py env)"; PICAR_ROS_NETWORK=host \\
    python evaluations/dock-3156/live.py [out.jsonl]

(`demo_explore.stack()` takes the desktop's exclusive lock itself.)

* C1-C3: per house (scaled_house, home_first_floor), one stack, five
  missions; before each, a person's nav2 goal to a spread start (that pose is
  the dock, D1). Finds of the red backpack and surveys in turn, the time
  limit `LIMIT_S`. Per mission: the outcome, `return`, the true pose error
  at the dock against the dock the brain recorded, contacts (chassis gap
  < 1 cm) and the minimum gap over the return, the return's reversals.
* C7 (scaled_house): `DRILLS` missions with a short limit, so they return
  early; once the return is driving, a person's Stop (`POST /stop`, no
  driver: ranked a person) or a D-pad FORWARD (`x-driver: twin-dpad`), in
  turn. A Stop: the return `cancelled`, the outcome unchanged, true
  displacement over the next 10 s <= 5 cm (3.84's live bar). A D-pad: the
  return `cancelled` (`preempted`).
"""

import json
import math
import os
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import httpx  # noqa: E402

from sim.grid_world import CELL_WALL  # noqa: E402
from sim.maps import build_world  # noqa: E402
from tests import demo_explore as de  # noqa: E402
from tests.footprint_sweep import CELL_CM, chassis, gap, square  # noqa: E402

HOUSES = ("scaled_house", "home_first_floor")
PER_HOUSE = 5
LIMIT_S = 600
DRILLS = 20                 # 10 Stops and 10 D-pads, in turn
DRILL_LIMIT_S = 120
WALL_S = 900                # one mission's wall-clock cap (the limit plus the return)


def spread_cells(house, n, seed=0):
    w = build_world(house)
    reach = de.reachable_cells(w)
    ok = [c for c in reach if all((c[0] + dx, c[1] + dy) in reach
                                  for dx in (-2, -1, 0, 1, 2) for dy in (-2, -1, 0, 1, 2))]
    rng = random.Random(f"dock-live-{house}-{seed}")
    picked = [rng.choice(ok)]
    while len(picked) < n:
        picked.append(max(ok, key=lambda c: min(math.hypot(c[0] - p[0], c[1] - p[1])
                                                for p in picked)))
    return picked


class Truth:
    def __init__(self, robot, house):
        self.robot = robot
        w = build_world(house)
        self.walls = {(x, y) for y, row in enumerate(w.layout) for x, c in enumerate(row)
                      if c == CELL_WALL}

    def pose(self):
        t = self.robot.get("/world/truth").json()
        return t["x_m"], t["y_m"], t["heading_deg"]

    def gap_cm(self):
        x, y, h = self.pose()
        cx, cy = x / 0.3, y / 0.3
        objs = {(o["x"], o["y"]) for o in self.robot.get("/sim/objects").json()["objects"]}
        A = chassis(cx, cy, math.radians(h) - math.pi / 2)
        g = math.inf
        for c in self.walls | objs:
            if abs(c[0] + 0.5 - cx) > 1.6 or abs(c[1] + 0.5 - cy) > 1.6:
                continue
            g = min(g, gap(A, square(*c)) * CELL_CM)
        return g


def goto(robot, x_m, y_m, timeout_s=240):
    """A person's nav2 goal (tap-to-goal, no driver: ranked a person)."""
    r = robot.post("/world/goal", json={"x_m": x_m, "y_m": y_m})
    if r.status_code != 200 or not r.json().get("accepted"):
        return f"refused: {r.status_code} {r.text[:120]}"
    end = time.time() + timeout_s
    while time.time() < end:
        g = (robot.get("/world/goal").json().get("goal") or {})
        if g.get("state") not in ("pending", "active"):
            return g.get("state")
        time.sleep(1.0)
    robot.delete("/world/goal")
    return "timeout"


def mission(robot, brain, truth, body, wall_s=WALL_S, drill=None):
    r = brain.post("/mission/start", json=body)
    if r.status_code != 200:
        return {"start": r.status_code, "detail": r.text[:300]}
    t0 = time.time()
    rec = {"body": body, "contacts": 0, "min_gap_cm": math.inf, "return_min_gap_cm": math.inf,
           "samples": 0}
    s = {}
    while time.time() - t0 < wall_s:
        g = truth.gap_cm()
        rec["samples"] += 1
        rec["min_gap_cm"] = min(rec["min_gap_cm"], g)
        rec["contacts"] += g < 1.0
        s = brain.get("/mission/status").json()
        ret = s.get("return") or {}
        if ret.get("state") == "returning":
            rec["return_min_gap_cm"] = min(rec["return_min_gap_cm"], g)
            if drill and "drill_at" not in rec and ret.get("phase") in ("driving", "creep", "turn"):
                rec.update(drill_at=round(time.time() - t0, 1), drill_kind=drill)
                p0 = truth.pose()
                if drill == "stop":
                    robot.post("/stop")
                    t1 = time.time()
                    moved = 0.0
                    while time.time() - t1 < 10.0:
                        p = truth.pose()
                        moved = max(moved, math.hypot(p[0] - p0[0], p[1] - p0[1]))
                        time.sleep(0.2)
                    rec["moved_after_stop_cm"] = round(moved * 100, 1)
                else:
                    robot.post("/action", json={"action": "FORWARD", "speed": 30, "duration": 0.5},
                               headers={"x-driver": "twin-dpad"})
        if not s.get("running") and s.get("outcome") not in (None, "idle", "running"):
            break
        time.sleep(0.5)
    else:
        brain.post("/mission/stop")
        rec["wall_capped"] = True
    ret = s.get("return") or {}
    dock = s.get("dock") or {}
    x, y, h = truth.pose()
    rec.update(outcome=s.get("outcome"), end_reason=(s.get("end_reason") or "")[:160],
               ret=ret, dock=dock, seconds=round(time.time() - t0),
               home=((s.get("budget") or {}).get("home")))
    if dock:
        rec["err_m"] = round(math.hypot(x - dock["x_m"], y - dock["y_m"]), 3)
        rec["err_deg"] = round(abs((h - dock["theta"] + 180) % 360 - 180), 1)
    for k in ("min_gap_cm", "return_min_gap_cm"):
        rec[k] = None if rec[k] == math.inf else round(rec[k], 1)
    return rec


def main(argv):
    out = Path(argv[0]) if argv else Path(__file__).with_name("live.jsonl")
    rows = []
    # Resumable: a run stopped part-way (the desktop wanted elsewhere) skips
    # every mission already recorded in `out`.
    done = set()
    if out.exists():
        for line in out.read_text().splitlines():
            r = json.loads(line)
            done.add((r["kind"], r["house"], tuple(r["start_cell"])))

    def emit(r):
        rows.append(r)
        print(json.dumps(r), flush=True)
        with open(out, "a") as f:
            f.write(json.dumps(r) + "\n")

    for house in HOUSES + ("drills",):
        h = "scaled_house" if house == "drills" else house
        kind = "c7" if house == "drills" else "c1"
        cells = (spread_cells(h, DRILLS, seed=1) if house == "drills"
                 else spread_cells(h, PER_HOUSE))
        if all((kind, h, tuple(c)) in done for c in cells):
            continue
        robot = de.stack(h)
        try:
            brain = httpx.Client(base_url=f"http://127.0.0.1:{de.BRAIN}", timeout=60)
            truth = Truth(robot, h)
            if house != "drills":
                for i, c in enumerate(cells):
                    if (kind, h, tuple(c)) in done:
                        continue
                    arrived = goto(robot, (c[0] + 0.5) * 0.3, (c[1] + 0.5) * 0.3)
                    time.sleep(3.0)          # the person's authority lapses
                    body = ({"job": "survey", "policy": "explore"} if i % 2
                            else {"target_object": de.TARGET, "policy": "explore"})
                    body.update(max_steps=400, mission_time_limit_s=LIMIT_S)
                    emit({"kind": "c1", "house": h, "start_cell": c, "start_goal": arrived,
                          **mission(robot, brain, truth, body)})
            else:
                for i in range(DRILLS):
                    c = cells[i]
                    if (kind, h, tuple(c)) in done:
                        continue
                    arrived = goto(robot, (c[0] + 0.5) * 0.3, (c[1] + 0.5) * 0.3)
                    time.sleep(3.0)
                    body = {"job": "survey", "policy": "explore", "max_steps": 400,
                            "mission_time_limit_s": DRILL_LIMIT_S}
                    emit({"kind": "c7", "house": h, "start_cell": c, "start_goal": arrived,
                          **mission(robot, brain, truth, body,
                                    drill="stop" if i % 2 == 0 else "dpad")})
        finally:
            de._kill_ports()
            os.system(f"docker rm -f {de.CONTAINER} >/dev/null 2>&1")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
