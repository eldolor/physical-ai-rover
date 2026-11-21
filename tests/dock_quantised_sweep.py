# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/dock_quantised_sweep.py -- 3.198's in-process harness
(`docs/plans/ros-alignment/3.198-return-goal-race.md`, Q2-Q4).

    .venv/bin/python tests/dock_quantised_sweep.py <arm> <out.json> [workers]

3.156.B's `tests/dock_sweep.py` (its starts, its InflatedNav, `MissionRunner`
-> explore -> `robot/safety.py` -> `MockRobot`), the 60 house missions
(`scaled_house`, `home_first_floor`, `complex_house` x 10 starts x {find,
survey}), 1200 s limit, 12.0 V, no drain. Arms:

* `q10`: whole-cell verbs (`MockRobot` without `exact_verbs`: a straight verb
  is whole 0.30 m moves, as `robot/ros_drive.py` makes it under `drive:
  ros`), goals ended at 1/3 of a cell (nav2's `xy_goal_tolerance` 0.10 m).
* `q12`: the same, goals ended at InflatedNav's own 0.4 cell (0.12 m).
* `exact`: `exact_verbs` and 0.4 cell: 3.156's `on` arm on these starts (Q4).

Per mission, beside dock_sweep's row: the ReturnTrip phases it reached, its
correcting goals (3.198 D2), its verbs in the approach and the longest run of
turns from one place, and the true error at the end. Run from a `dev`
checkout (with this file and `tests/drop_sweep.py`'s STOP_CELLS) for dev's
numbers in the same arms.
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tests.dock_sweep import CELL, HOUSES, LIMIT_S, STARTS, TICK_CAP, inflated_nav_class, spread_starts  # noqa: E402

ARMS = {"q10": (False, 1.0 / 3.0), "q12": (False, 0.4), "exact": (True, 0.4)}


def cases():
    return [(h, s, job) for h in HOUSES for s in spread_starts(h, STARTS) for job in ("find", "survey")]


def run_one(args):
    import logging
    logging.disable(logging.WARNING)
    from control.mission_runner import MissionRunner
    from sim.battery import SimBattery
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    arm, house, start, job = args
    exact, stop_cells = ARMS[arm]
    grid = build_world(house)
    grid.x, grid.y = start[0], start[1]
    grid.theta = math.radians(start[2] - 90)
    bat = SimBattery(v0=12.0, drain_v_per_s=0.0, clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat, exact_verbs=exact)
    nav_cls = inflated_nav_class()
    nav_cls.STOP_CELLS = stop_cells
    nav = nav_cls(robot)
    sent = []
    set_goal = nav.set_goal

    def counted_goal(x_m, y_m):
        sent.append((round(x_m, 3), round(y_m, 3), runner._return_trip is not None))
        return set_goal(x_m, y_m)
    nav.set_goal = counted_goal
    kw = {"job": "survey"} if job == "survey" else {"target_object": "red backpack"}
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(grid),
                           mission_time_limit_s=LIMIT_S, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=True, **kw)
    verbs = []
    safety = runner.agent.safety
    check = safety.check_and_execute

    def recorded(action, **kw):
        if runner._return_trip is not None:
            verbs.append((action, round(grid.x, 4), round(grid.y, 4), runner._return_trip.state))
        return check(action, **kw)
    safety.check_and_execute = recorded
    runner.start()
    dock = (grid.x, grid.y, grid.heading_deg)
    phases, ticks = set(), 0
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
        if runner._return_trip is not None:
            phases.add(runner._return_trip.state)
    s = runner.status()
    ret = s.get("return") or {}
    approach = [v for v in verbs if v[3] in ("turn", "creep", "final")]
    run = best = 0
    last = None
    for a, x, y, _ in verbs:
        if a in ("LEFT", "RIGHT"):
            run = run + 1 if last == (x, y) else 1
            last = (x, y)
            best = max(best, run)
        else:
            run, last = 0, None
    return {"arm": arm, "house": house, "start": start, "job": job,
            "outcome": s["outcome"], "return": ret.get("state"), "return_reason": ret.get("reason"),
            "reached_approach": bool(phases & {"turn", "creep", "final"}),
            "fine_goals": ret.get("fine_goals"), "return_goals": sum(g[2] for g in sent),
            "approach_verbs": len(approach), "turn_run_max": best,
            "err_m": round(math.hypot((grid.x - dock[0]) * CELL, (grid.y - dock[1]) * CELL), 4),
            "err_deg": round(abs((grid.heading_deg - dock[2] + 180) % 360 - 180), 2),
            "end_s": round(grid.sim_time, 1), "ticks": ticks, "capped": ticks >= TICK_CAP}


def verdict(rows):
    reached = [r for r in rows if r["reached_approach"]]
    docked = [r for r in reached if r["return"] == "docked"]
    errs = sorted(r["err_m"] for r in docked)
    return {"missions": len(rows), "returns": sum(r["return"] not in (None, "skipped") for r in rows),
            "reached_approach": len(reached), "docked": len(docked),
            "docked_share": round(len(docked) / len(reached), 3) if reached else None,
            "docked_truth_ok": sum(r["err_m"] <= 0.10 and r["err_deg"] <= 10.0 for r in docked),
            "not_aligned_bare": sum(r["return_reason"] == "not_aligned" for r in rows),
            "not_docked": [(r["house"], r["job"], r["return"], r["return_reason"]) for r in reached
                           if r["return"] != "docked"][:20],
            "approach_verbs_max": max((r["approach_verbs"] for r in reached), default=None),
            "over_12_verbs": sum(r["approach_verbs"] > 12 for r in reached),
            "turn_run_max": max((r["turn_run_max"] for r in rows), default=None),
            "fine_goals": sum(r["fine_goals"] or 0 for r in rows),
            "err_m_median": errs[len(errs) // 2] if errs else None,
            "err_m_max": errs[-1] if errs else None}


def _key(r):
    return json.dumps([r["house"], list(r["start"]), r["job"]])


def main(argv):
    import multiprocessing as mp
    arm, out_path = argv[0], argv[1]
    workers = int(argv[2]) if len(argv) > 2 else 4
    part = Path(out_path + ".rows.jsonl")
    done = {}
    if part.exists():
        for line in part.read_text().splitlines():
            r = json.loads(line)
            done[_key(r)] = r
    args = [(arm,) + c for c in cases() if json.dumps([c[0], list(c[1]), c[2]]) not in done]
    with mp.Pool(workers) as pool, open(part, "a") as f:
        for r in pool.imap_unordered(run_one, args, chunksize=1):
            done[_key(r)] = r
            f.write(json.dumps(r) + "\n")
            f.flush()
    rows = sorted(done.values(), key=_key)
    summary = verdict(rows)
    Path(out_path).write_text(json.dumps({"arm": arm, "summary": summary, "rows": rows}) + "\n")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
