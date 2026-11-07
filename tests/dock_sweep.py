# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
tests/dock_sweep.py -- 3.156.B's in-process harness
(`docs/plans/ros-alignment/3.156-return-to-dock.md`, criteria C1, C4-C6).

    .venv/bin/python tests/dock_sweep.py <arm> <out.json> [workers]

`MissionRunner` -> the explore policy -> `robot/safety.py` -> `MockRobot`,
nav2 stood in for by `tests/drop_sweep.py`'s VettedNav (it plans over the
true floor and DRIVES through the veto, `vet_wheel_velocity()` then
`advance()`), the dock the start pose, `mission_time_limit_s` 1200 (the
config's) and `battery_floor_v` 10.5 with a battery. Starts: scaled_house,
home_first_floor and complex_house x 10 spread starts x {find the red
backpack, survey}, plus ProcTHOR val 0-29 (its own start, survey): 90.

Arms: `on` (the return, 12.0 V, no drain), `off` (return_to_dock off, the
same starts: C6's pair), `drain-0.005`, `drain-0.01`, `drain-0.02` (11.0 V
falling at that many V/s; the first 30 starts: C5).

Ground truth per mission: the outcome and when it was decided, the return's
state and reason, the true pose error at the end, metres driven home against
the true floor's shortest path (C4), the trip's time against D3's predicted
`T_home` at the decision, and the true voltage when the return ended (C5:
Reserve is 10.2 V).
"""

import json
import math
import random
import sys
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

HOUSES = ("scaled_house", "home_first_floor", "complex_house")
PROCTHOR = tuple(f"procthor:val:{i}" for i in range(30))
STARTS = 10
LIMIT_S = 1200.0
FLOOR_V = 10.5
RESERVE_V = 10.2
CELL = 0.30
TICK_CAP = 20000


def spread_starts(house, n, seed=0):
    """`n` free floor cells spread over the house (farthest-point, seeded),
    each with a seeded heading; at least 2 cells from any wall or object."""
    from sim.grid_world import CELL_WALL
    from sim.maps import build_world
    w = build_world(house)
    free = []
    for y, row in enumerate(w.layout):
        for x, c in enumerate(row):
            if c != "." or (x, y) in w.objects:
                continue
            if all(w._cell(x + dx, y + dy) != CELL_WALL and (x + dx, y + dy) not in w.objects
                   for dx in (-1, 0, 1) for dy in (-1, 0, 1)):
                free.append((x, y))
    rng = random.Random(f"dock-{house}-{seed}")
    picked = [rng.choice(free)]
    while len(picked) < n:
        picked.append(max(free, key=lambda c: min(math.hypot(c[0] - p[0], c[1] - p[1])
                                                  for p in picked)))
    return [(x + 0.5, y + 0.5, rng.randrange(0, 360, 90)) for x, y in picked]


def cases(arm):
    out = [(h, s, job) for h in HOUSES for s in spread_starts(h, STARTS)
           for job in ("find", "survey")]
    out += [(h, None, "survey") for h in PROCTHOR]
    if arm.startswith("drain-"):
        out = out[:30]
    return out


def inflated_nav_class():
    """VettedNav planning as nav2 does on its inflated costmap: through cells
    with a cell of clearance on every side (0.30 m; nav2's inflation is
    0.35 m), so its pivots are not vetoed against the walls it hugged; its
    own plain path when no such path exists (a door, a tight start)."""
    from sim.grid_world import CELL_WALL
    from tests.drop_sweep import _vetted_nav_class
    base = _vetted_nav_class()

    class InflatedNav(base):
        def _clear(self, c, ends):
            g = self.grid
            if c in ends:
                return True
            return all(g._cell(c[0] + dx, c[1] + dy) != CELL_WALL
                       and (c[0] + dx, c[1] + dy) not in g.objects
                       for dx in (-1, 0, 1) for dy in (-1, 0, 1))

        def _path(self, a, b):
            ends, prev, q = {a, b}, {a: None}, deque([a])
            while q:
                c = q.popleft()
                if c == b:
                    out = []
                    while c is not None:
                        out.append(c)
                        c = prev[c]
                    return out[::-1]
                for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
                    if n not in prev and self._clear(n, ends):
                        prev[n] = c
                        q.append(n)
            return super()._path(a, b)

    return InflatedNav


def true_path_m(grid, a, b):
    """Shortest 4-connected path over the true floor, in metres (None: none)."""
    from sim.grid_world import CELL_WALL
    a, b = (math.floor(a[0]), math.floor(a[1])), (math.floor(b[0]), math.floor(b[1]))
    seen, q = {a: 0}, deque([a])
    while q:
        c = q.popleft()
        if c == b:
            return seen[c] * CELL
        for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
            if n not in seen and grid._cell(*n) != CELL_WALL and n not in grid.objects:
                seen[n] = seen[c] + 1
                q.append(n)
    return None


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
    drain = float(arm.split("-", 1)[1]) if arm.startswith("drain-") else 0.0
    bat = SimBattery(v0=11.0 if drain else 12.0, drain_v_per_s=drain,
                     clock=lambda: grid.sim_time)
    robot = MockRobot(grid, render=False, battery=bat, exact_verbs=True)
    nav = inflated_nav_class()(robot)
    kw = {"job": "survey"} if job == "survey" else {"target_object": "red backpack"}
    runner = MissionRunner(robot, policy="explore", navigator=nav, world=MockWorld(grid),
                           mission_time_limit_s=LIMIT_S, battery_floor_v=FLOOR_V,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           return_to_dock=arm != "off", **kw)
    rooms = set()
    step = robot.step

    def counted(dt):
        r = step(dt)
        rooms.add(grid.room_at(math.floor(grid.x), math.floor(grid.y)))
        return r
    robot.step = counted
    runner.start()
    dock = (grid.x, grid.y, grid.heading_deg)
    ticks, decided = 0, None
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
        if decided is None and runner._pending_end is not None:
            p = runner.world.get_pose()
            decided = {"t": round(grid.sim_time, 1), "x": grid.x, "y": grid.y,
                       "home": dict(runner._home.last) if runner._home else {},
                       "rooms": len(rooms - {"unknown"})}
    s = runner.status()
    ret = s.get("return") or {}
    err_m = math.hypot((grid.x - dock[0]) * CELL, (grid.y - dock[1]) * CELL)
    err_deg = abs((grid.heading_deg - dock[2] + 180) % 360 - 180)
    row = {"arm": arm, "house": house, "start": start, "job": job,
           "outcome": s["outcome"], "end_reason": (s.get("end_reason") or "")[:160],
           "return": ret.get("state"), "return_reason": ret.get("reason"),
           "decided_s": decided["t"] if decided else round(grid.sim_time, 1),
           "end_s": round(grid.sim_time, 1), "ticks": ticks, "capped": ticks >= TICK_CAP,
           "rooms": decided["rooms"] if decided else len(rooms - {"unknown"}),
           "err_m": round(err_m, 3), "err_deg": round(err_deg, 1),
           "v_end": round(bat.true_v(), 3), "reserve_crossed": bat.true_v() < RESERVE_V,
           "reversals": ret.get("reversals"), "actions": ret.get("actions"),
           "metres_home": ret.get("metres")}
    if decided:
        row["home_pred"] = decided["home"]
        row["path_true_m"] = true_path_m(grid, (decided["x"], decided["y"]), dock)
        row["trip_s"] = round(grid.sim_time - decided["t"], 1)
    return row


def verdict(rows):
    def share(n, d):
        return round(n / d, 3) if d else None
    out = {}
    on = [r for r in rows if r["arm"] == "on"]
    returns = [r for r in on if r["return"] not in (None, "skipped") or
               (r["return"] == "skipped" and r["return_reason"] not in ("person", "fault"))]
    if on:
        out["C1"] = {"returns": len(returns), "docked": sum(r["return"] == "docked" for r in returns),
                     "share": share(sum(r["return"] == "docked" for r in returns), len(returns)),
                     "not_docked": [(r["house"], r["job"], r["return"], r["return_reason"])
                                    for r in returns if r["return"] != "docked"][:20]}
    trips = [r for r in rows if r["return"] == "docked" and r.get("path_true_m")]
    if trips:
        ratio = [r["metres_home"] / r["path_true_m"] for r in trips if r["path_true_m"] > 0.3]
        timed = [r for r in trips if r.get("home_pred", {}).get("t_home_s")]
        out["C4"] = {"trips": len(trips),
                     "metres_ratio_max": round(max(ratio), 2) if ratio else None,
                     "metres_over_1.5": sum(x > 1.5 for x in ratio),
                     "time_within_pred": share(sum(r["trip_s"] <= r["home_pred"]["t_home_s"]
                                                   for r in timed), len(timed))}
    for arm in sorted({r["arm"] for r in rows if r["arm"].startswith("drain-")}):
        rs = [r for r in rows if r["arm"] == arm]
        out[f"C5 {arm}"] = {"runs": len(rs), "reserve_crossed": sum(r["reserve_crossed"] for r in rs),
                            "docked": sum(r["return"] == "docked" for r in rs),
                            "by_limit": share(sum(r["end_s"] <= LIMIT_S for r in rs), len(rs)),
                            "worst_over_s": max(r["end_s"] - LIMIT_S for r in rs)}
    if on:
        out["C5 time (on)"] = {"by_limit": share(sum(r["end_s"] <= LIMIT_S for r in on), len(on)),
                               "worst_over_s": max(r["end_s"] - LIMIT_S for r in on)}
    return out


def _key(r):
    return json.dumps([r["house"], r["start"], r["job"]])


def main(argv):
    """Each mission is appended to `<out>.rows.jsonl` as it ends, so a sweep
    stopped part-way (the desktop lock wanted elsewhere) resumes where it was:
    finished missions are not run again."""
    import multiprocessing as mp
    arm, out_path = argv[0], argv[1]
    workers = int(argv[2]) if len(argv) > 2 else 4
    part = Path(out_path + ".rows.jsonl")
    done = {}
    if part.exists():
        for line in part.read_text().splitlines():
            r = json.loads(line)
            done[_key(r)] = r
    args = [(arm,) + c for c in cases(arm)
            if json.dumps([c[0], list(c[1]) if c[1] else None, c[2]]) not in done]
    with mp.Pool(workers) as pool, open(part, "a") as f:
        for r in pool.imap_unordered(run_one, args, chunksize=1):
            done[_key(r)] = r
            f.write(json.dumps(r) + "\n")
            f.flush()
    rows = list(done.values())
    summary = verdict(rows)
    Path(out_path).write_text(json.dumps({"arm": arm, "summary": summary, "rows": rows}) + "\n")
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main(sys.argv[1:])
