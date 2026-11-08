# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.162.C's harness, run by hand (no tests in it):

    .venv/bin/python -m tests.test_what_i_saw_sweep <out.json>

In process, `MissionRunner` -> policy -> `robot/safety.py` -> `MockRobot`
with the renderer on, each report written by the real path
(`control.brain_server._write_saw` -> `ReportWriter` -> `ReportStore`):

* W1: the 12 clear starts of the tiered arrival rig, each to `found` and to
  `arrived_unconfirmed`; the report's frame hash against the frame the
  identity check was handed.
* W3 (live missions' sizes), W4: 3.46's 20 `complex_house` starts (frontier,
  a target that is not there), each listed thing against the house's truth.
* W7: those 20 with the sink set and unset, alternating; the tick's p90.

Timed: hold the desktop lock exclusively.
"""

import base64
import hashlib
import json
import logging
import math
import statistics
import sys
import tempfile
import time
from pathlib import Path

from brain import what_i_saw as wis
from brain.navigate import CloudUnavailable
from brain.perceive import FrameReportedPipeline
from brain.tiered import TieredVision
from control import brain_server
from control.mission_runner import ARRIVED_UNCONFIRMED, FOUND, MissionRunner
from sim.maps import build_world
from sim.maps import complex_house as house
from sim.mock_robot import MockRobot
from tests import inventory_sweep as inv_sweep
from tests.conftest import mock_world_for
from tests.test_bearing_turns import CLEAR_STARTS, GOAL, TARGET, _build, _quiet_cloud

CELL_M = 0.30
_ROOM_OF = {c: r for r, cells in house.ROOMS.items() for c in cells}


def _sink(writer, mission_id):
    return lambda payload: brain_server._write_saw(lambda: writer, mission_id, payload)


def w1_one(store, writer, i, start, down):
    x, y, off = start
    grid = _build()
    grid.x, grid.y = x, y
    grid.theta = math.atan2(GOAL[1] - y, GOAL[0] - x) + math.radians(off)
    robot = MockRobot(grid, render=True)
    tier = TieredVision(FrameReportedPipeline(TARGET), _quiet_cloud, steer_on_sight=True, hold_goal=True)
    sent = []
    real = tier.confirm_arrival

    def confirm(frame):
        sent.append(hashlib.sha256(base64.b64decode(frame["image_base64"])).hexdigest())
        if down:
            raise CloudUnavailable("cloud down")
        return real(frame)
    tier.confirm_arrival = confirm
    runner = MissionRunner(robot, target_object=TARGET, max_steps=120, policy="tiered", vision_fn=tier,
                           world=mock_world_for(robot))
    mid = f"w1-{i}-{int(down)}"
    runner.saw_sink = _sink(writer, mid)
    runner.start()
    while runner.tick():
        pass
    outcome = runner.status()["outcome"]
    if outcome not in (FOUND, ARRIVED_UNCONFIRMED):
        return {"mid": mid, "outcome": outcome, "reported": None}
    writer.flush()
    rep = store.get(mid)
    return {"mid": mid, "outcome": outcome, "shown": bool(rep.get("found")),
            "judged": bool(rep.get("found")) and bool(sent) and rep["found"]["sha256"] == sent[-1],
            "confirmed": (rep.get("found") or {}).get("confirmed"),
            "bytes": sum(p.stat().st_size for p in (store.dir / mid).rglob("*") if p.is_file())}


def sweep_one(store, writer, i, start, on):
    cell, heading, _ = start
    grid = build_world("complex_house")
    grid.x, grid.y = cell[0] + 0.5, cell[1] + 0.5
    grid.theta = heading
    robot = MockRobot(grid, render=True)
    runner = MissionRunner(robot, policy="frontier", target_object="unicorn", max_steps=inv_sweep.MAX_STEPS,
                           world=mock_world_for(robot))
    mid = f"sw-{i}"
    if on:
        runner.saw_sink = _sink(writer, mid)
    runner.start()
    ticks = []
    while True:
        t = time.perf_counter()
        more = runner.tick()
        ticks.append(time.perf_counter() - t)
        if not more:
            break
    out = {"ticks": ticks, "outcome": runner.status()["outcome"]}
    if not on:
        return out
    writer.flush()
    rep = store.get(mid)
    rows = []
    for m in rep["landmarks"]:
        best, near = None, math.inf
        for name, cells in house.INSTANCES:
            if name != m["label"]:
                continue
            for c in cells:
                d = math.hypot(max(abs(m["x_m"] / CELL_M - (c[0] + .5)) - .5, 0),
                               max(abs(m["y_m"] / CELL_M - (c[1] + .5)) - .5, 0)) * CELL_M
                if d < near:
                    best, near = c, d
        rows.append({"label": m["label"], "err_m": None if best is None else round(near, 3),
                     "room": m["room"], "truth_room": _ROOM_OF.get(best), "thumb": bool(m["thumb"])})
    out.update(rows=rows, bytes=sum(p.stat().st_size for p in (store.dir / mid).rglob("*") if p.is_file()))
    return out


def p90(xs):
    return statistics.quantiles(xs, n=10)[-1]


def main(path):
    logging.disable(logging.WARNING)
    with tempfile.TemporaryDirectory() as d:
        store = wis.ReportStore(d)
        writer = wis.ReportWriter(store)
        w1 = [w1_one(store, writer, i, s, down) for i, s in enumerate(CLEAR_STARTS) for down in (False, True)]
        starts = inv_sweep.starts()
        on_runs, off_runs = [], []
        for i, s in enumerate(starts):
            pair = [(True, on_runs), (False, off_runs)]
            for on, sink in (pair if i % 2 else pair[::-1]):
                sink.append(sweep_one(store, writer, i, s, on))
    on_t = [t * 1000 for r in on_runs for t in r["ticks"]]
    off_t = [t * 1000 for r in off_runs for t in r["ticks"]]
    rows = [r for run in on_runs for r in run["rows"]]
    placed = sorted(r["err_m"] for r in rows if r["err_m"] is not None)
    roomed = [r for r in rows if r["truth_room"]]
    ended = [r for r in w1 if r.get("shown") is not None]
    out = {"w1": {"runs": w1, "ended": len(ended), "shown": sum(1 for r in ended if r["shown"]),
                  "judged": sum(1 for r in ended if r["judged"]),
                  "outcomes": {o: sum(1 for r in w1 if r["outcome"] == o) for o in {r["outcome"] for r in w1}}},
           "w3": {"max_mission_bytes": max([r["bytes"] for r in on_runs] + [r.get("bytes", 0) for r in w1])},
           "w4": {"rows": len(rows), "no_such_label": sum(1 for r in rows if r["err_m"] is None),
                  "err_p90_m": p90(placed) if len(placed) > 1 else None,
                  "room_right": sum(1 for r in roomed if r["room"] == r["truth_room"]), "roomed": len(roomed),
                  "thumbs": sum(1 for r in rows if r["thumb"])},
           "w7": {"tick_p90_ms_on": round(p90(on_t), 2), "tick_p90_ms_off": round(p90(off_t), 2),
                  "ticks_on": len(on_t), "ticks_off": len(off_t),
                  "outcomes_same": [r["outcome"] for r in on_runs] == [r["outcome"] for r in off_runs]}}
    Path(path).write_text(json.dumps(out, indent=1))
    print(json.dumps({k: v for k, v in out.items() if k != "w1"} | {"w1": {k: v for k, v in out["w1"].items() if k != "runs"}}, indent=1))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
