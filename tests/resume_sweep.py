# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/resume_sweep.py -- 3.156.D's in-process harness
(`docs/plans/ros-alignment/3.156-return-to-dock.md`, criteria E1, E2, E4-E6).

    .venv/bin/python tests/resume_sweep.py <arm> <out.json> [workers]

`tests/dock_sweep.py`'s path (`MissionRunner` -> explore -> `robot/safety.py`
-> `MockRobot`, its InflatedNav for nav2) over its first 30 starts, 12.0 V at
the start, 0.02 V/s on the charger. Arms: `base` (no drain), `charge-<V/s>`,
`nocharge-<V/s>` (B's end). Resumable, as `dock_sweep`.
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tests import dock_sweep as ds  # noqa: E402

CHARGE_V_S = 0.02
RESERVE_V = 10.2


def run_one(args):
    import logging
    logging.disable(logging.WARNING)
    from control.mission_runner import MissionRunner
    from sim.battery import SimBattery
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    arm, house, start, job = args
    grid = build_world(house)
    if start is not None:
        grid.x, grid.y = start[0], start[1]
        grid.theta = math.radians(start[2] - 90)
    drain = float(arm.split("-", 1)[1]) if "-" in arm else 0.0
    bat = SimBattery(v0=12.0, drain_v_per_s=drain, charge_v_per_s=CHARGE_V_S,
                     clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat, exact_verbs=True)
    nav = ds.inflated_nav_class()(robot)
    kw = {"job": "survey"} if job == "survey" else {"target_object": "red backpack"}
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(grid),
                           mission_time_limit_s=ds.LIMIT_S, battery_floor_v=ds.FLOOR_V,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           return_to_dock=True, recharge_resume=arm.startswith("charge"), **kw)
    rooms, min_v = set(), [bat.true_v()]
    step = robot.step

    def counted(dt):
        r = step(dt)
        rooms.add(grid.room_at(math.floor(grid.x), math.floor(grid.y)))
        min_v[0] = min(min_v[0], bat.true_v())
        return r
    robot.step = counted
    runner.start()
    ticks = 0
    while runner.tick() and ticks < ds.TICK_CAP * 3:
        ticks += 1
    s = runner.status()
    budget, charge, ret = s["budget"], s.get("charge") or {}, s.get("return") or {}
    return {"arm": arm, "house": house, "start": start, "job": job,
            "outcome": s["outcome"], "end_reason": (s.get("end_reason") or "")[:160],
            "return": ret.get("state"), "return_reason": ret.get("reason"),
            "charge": charge.get("state"), "charge_reason": charge.get("reason"),
            "history": charge.get("history") or [],
            "elapsed_s": budget["elapsed_s"], "paused_s": budget["paused_s"],
            "rooms": len(rooms - {"unknown"}), "min_v": round(min_v[0], 3),
            "ticks": ticks, "capped": ticks >= ds.TICK_CAP * 3}


def verdict(rows):
    out = {}
    charged = [r for r in rows if r["arm"].startswith("charge")]
    battery_ends = [r for r in charged if r["history"] or r["charge"]]
    docked = [r for r in battery_ends if r["charge"] != "not_docked"]
    out["E1"] = {"battery_ends": len(battery_ends), "docked": len(docked),
                 "resumed": sum(bool(r["history"]) for r in docked),
                 "states": sorted({str(r["charge"]) for r in charged})}
    limit = ds.LIMIT_S + 60.0
    out["E4"] = {"over_limit": sum(r["elapsed_s"] - r["paused_s"] > limit for r in rows),
                 "charges_not_from_the_battery": sum("battery" not in h["after"] for r in rows
                                                     for h in r["history"])}
    out["E5"] = {"at_or_under_reserve": sum(r["min_v"] <= RESERVE_V for r in charged),
                 "n": len(charged), "worst_v": min((r["min_v"] for r in charged), default=None)}
    out["E6"] = {"max_charges": max((len(r["history"]) for r in rows), default=0)}
    arms = {}
    for r in rows:
        a = arms.setdefault(r["arm"], {"finds": 0, "survey_rooms": 0, "n": 0, "capped": 0})
        a["n"] += 1
        a["capped"] += r["capped"]
        a["finds"] += r["outcome"] == "found"
        if r["job"] == "survey":
            a["survey_rooms"] += r["rooms"]
    out["arms"] = arms
    return out


def main(argv):
    import multiprocessing as mp
    arm, out_path = argv[0], argv[1]
    workers = int(argv[2]) if len(argv) > 2 else 4
    part = Path(out_path + ".rows.jsonl")
    done = {}
    if part.exists():
        for line in part.read_text().splitlines():
            r = json.loads(line)
            done[ds._key(r)] = r
    cases = ds.cases("on")[:30]
    args = [(arm,) + c for c in cases
            if json.dumps([c[0], list(c[1]) if c[1] else None, c[2]]) not in done]
    with mp.Pool(workers) as pool, open(part, "a") as f:
        for r in pool.imap_unordered(run_one, args, chunksize=1):
            done[ds._key(r)] = r
            f.write(json.dumps(r) + "\n")
            f.flush()
    rows = list(done.values())
    Path(out_path).write_text(json.dumps({"arm": arm, "summary": verdict(rows), "rows": rows}) + "\n")
    print(json.dumps(verdict(rows), indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
