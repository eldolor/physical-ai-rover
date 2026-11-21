# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/dock_late_sweep.py -- 3.194's in-process sweep
(`docs/plans/ros-alignment/3.194-late-dock.md`, criteria L2-L4).

    .venv/bin/python tests/dock_late_sweep.py <arm> <out.json> [workers]

The real path -- `MissionRunner` -> the explore policy -> `robot/safety.py` ->
`MockRobot` -- with 3.156's driving navigator (`tests/dock_sweep.py`'s
InflatedNav: nav2 stood in for, every move through the veto and the wheels,
so the odometry sees it). The robot starts on the dock. Starts:
scaled_house, home_first_floor and complex_house x 10 spread starts (3.156's)
x {find the red backpack, survey}: 60. `mission_time_limit_s` 1200.

Arms (the world given to the runner, so explore sees the same pose):
* `base`: the pose usable from the first read, no saved dock (L4: compared
  mission by mission with the same arm run on `dev`).
* `saved-N`: the first N pose reads unusable, then every pose carries the
  saved map's dock (3.156.E's `dock`), the true start (L2, L3).
* `start-N`: the first N reads unusable, no saved dock (L3: a start pose
  read after the robot moved is never used).

Ground truth per mission: the dock the runner used against the true dock,
when the pose first became usable, and the return's state and reason.
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

HOUSES = ("scaled_house", "home_first_floor", "complex_house")
STARTS = 10
LIMIT_S = 1200.0
CELL = 0.30
TICK_CAP = 20000


def cases():
    from tests.dock_sweep import spread_starts
    return [(h, s, job) for h in HOUSES for s in spread_starts(h, STARTS)
            for job in ("find", "survey")]


class LatePose:
    """The first `n` pose reads unusable; then the real pose, carrying
    `saved` (the saved map's dock) when given. Records when it first
    answered usable (the sim clock)."""

    def __init__(self, world, n, saved, clock):
        self.world, self.n, self.saved, self.clock = world, n, saved, clock
        self.calls, self.usable_at = 0, None

    def __getattr__(self, name):
        return getattr(self.world, name)

    def get_pose(self):
        from world.interface import unusable_pose
        self.calls += 1
        if self.calls <= self.n:
            return unusable_pose()
        pose = self.world.get_pose()
        if pose.get("usable") and self.usable_at is None:
            self.usable_at = self.clock()
        if self.saved is not None and pose.get("usable"):
            pose = {**pose, "dock": dict(self.saved)}
        return pose


def run_one(args):
    import logging
    logging.disable(logging.WARNING)
    from control.mission_runner import MissionRunner
    from sim.battery import SimBattery
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    from tests.dock_sweep import inflated_nav_class
    arm, house, start, job = args
    grid = build_world(house)
    grid.x, grid.y = start[0], start[1]
    grid.theta = math.radians(start[2] - 90)
    bat = SimBattery(v0=12.0, drain_v_per_s=0.0, clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat, exact_verbs=True)
    nav = inflated_nav_class()(robot)
    true = (grid.x * CELL, grid.y * CELL, grid.heading_deg)
    kind, _, n = arm.partition("-")
    saved = {"x_m": true[0], "y_m": true[1], "heading_deg": true[2]} if kind == "saved" else None
    world = MockWorld(grid)
    if kind != "base":
        world = LatePose(world, int(n), saved, lambda: grid.sim_time)
    kw = {"job": "survey"} if job == "survey" else {"target_object": "red backpack"}
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=world,
                           mission_time_limit_s=LIMIT_S, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=True, **kw)
    t0 = grid.sim_time
    runner.start()
    ticks, decided_s = 0, None
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
        if decided_s is None and runner._pending_end is not None:
            decided_s = round(grid.sim_time - t0, 1)
    s = runner.status()
    ret = s.get("return") or {}
    dock = s.get("dock")
    row = {"arm": arm, "house": house, "start": list(start), "job": job,
           "outcome": s["outcome"], "return": ret.get("state"),
           "return_reason": ret.get("reason"), "ticks": ticks, "capped": ticks >= TICK_CAP,
           "decided_s": decided_s if decided_s is not None else round(grid.sim_time - t0, 1),
           "end_s": round(grid.sim_time - t0, 1),
           "dock": dock,
           "end_err_m": round(math.hypot(grid.x * CELL - true[0], grid.y * CELL - true[1]), 3),
           "end_err_deg": round(abs((grid.heading_deg - true[2] + 180) % 360 - 180), 1)}
    if dock:
        row["dock_err_m"] = round(math.hypot(dock["x_m"] - true[0], dock["y_m"] - true[1]), 4)
        row["dock_err_deg"] = round(abs((dock["theta"] - true[2] + 180) % 360 - 180), 2)
    if isinstance(world, LatePose):
        row["usable_at_s"] = (round(world.usable_at - t0, 1)
                              if world.usable_at is not None else None)
    return row


def verdict(rows):
    out = {}
    for arm in sorted({r["arm"] for r in rows}):
        rs = [r for r in rows if r["arm"] == arm]
        used = [r for r in rs if r["dock"]]
        knowable = [r for r in rs if arm.startswith("saved") and r.get("usable_at_s") is not None
                    and r["usable_at_s"] <= r["decided_s"]]
        went = [r for r in knowable if r["return"] not in ("skipped", None)]
        out[arm] = {
            "missions": len(rs),
            "outcomes": {o: sum(r["outcome"] == o for r in rs) for o in
                         sorted({r["outcome"] for r in rs})},
            "returns": {k: sum(f"{r['return']}:{r['return_reason'] or ''}" == k for r in rs)
                        for k in sorted({f"{r['return']}:{r['return_reason'] or ''}"
                                         for r in rs})},
            "docks_used": len(used),
            "late": sum("late_s" in (r["dock"] or {}) for r in rs),
            "dock_err_max_m": max((r["dock_err_m"] for r in used), default=None),
            "dock_err_max_deg": max((r["dock_err_deg"] for r in used), default=None),
            "wrong_dock": sum(r["dock_err_m"] > 0.01 or r["dock_err_deg"] > 0.1 for r in used),
            "capped": sum(r["capped"] for r in rs),
        }
        if arm.startswith("saved"):
            out[arm]["knowable"] = len(knowable)
            out[arm]["return_ran"] = len(went)
            out[arm]["no_dock_when_knowable"] = sum(r["return_reason"] == "no_dock"
                                                    for r in knowable)
    return out


def _key(r):
    return json.dumps([r["arm"], r["house"], list(r["start"]), r["job"]])


def main(argv):
    """Each mission is appended to `<out>.rows.jsonl` as it ends; a sweep
    stopped part-way resumes where it was."""
    import multiprocessing as mp
    arm, out_path = argv[0], argv[1]
    workers = int(argv[2]) if len(argv) > 2 else 4
    part = Path(out_path + ".rows.jsonl")
    done = {}
    if part.exists():
        for line in part.read_text().splitlines():
            r = json.loads(line)
            done[_key(r)] = r
    args = [(arm,) + c for c in cases()
            if json.dumps([arm, c[0], list(c[1]), c[2]]) not in done]
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
