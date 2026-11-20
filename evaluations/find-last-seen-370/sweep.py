# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/find-last-seen-370/sweep.py -- 3.70.M2: "find X" looks first where
X was last seen. In process, as 3.175.B's sweep (`MissionRunner` -> explore ->
`robot/safety.py` -> `MockRobot` on the simulator's clock, `InflatedNav`, a real
`InventoryStore` chain, one saved map's `frame_id` on every pose). How the bars
are read is in `docs/plans/ros-alignment/3.70-survey-and-mission-ends.md`,
"M2: how it is measured", written before any run.

One survey of `complex_house` from its dock, then 20 finds for small objects
the survey reported, from spread starts, twice: *surveyed* (a copy of the
survey's store and its end map) and *unsurveyed* (nothing), the same
(start, target, detector seed) in both.

    python tools/desktop_lock.py shared --why "3.70.M2 sweep" -- \\
        python evaluations/find-last-seen-370/sweep.py run ROWS.jsonl WORKDIR [WORKERS [NAV]]
    (NAV: inflated, run 1; fake, amendment 1's run 2)
    python evaluations/find-last-seen-370/sweep.py score ROWS.jsonl
"""

import json
import math
import random
import shutil
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

HOUSE = "complex_house"
FRAME = "map-370m2"
SURVEY_LIMIT_S = 1800.0    # S1's survey limit
LIMIT_S = 1200.0           # 3.175's find limit
CELL = 0.30
FOUND_M = 0.9              # 3.80.G2: a find within this of the target is right
RECALL_AT_M = 0.30         # the first recall's place: one cell from a reported landmark of X
MISSIONS = 20
START_SEED = 3702          # fresh: S3 judged on 3700
DET_SEED = 37020
TICK_CAP = 40000
ARMS = ("surveyed", "unsurveyed")
NAV = "inflated"           # amendment 1: "fake" -- tests/test_explore.FakeNav, run 2


def _nav(robot, nav):
    if nav == "fake":
        from tests.test_explore import FakeNav
        return FakeNav(robot)
    from tests.dock_sweep import inflated_nav_class
    return inflated_nav_class()(robot)


def _quiet():
    import logging
    logging.disable(logging.WARNING)


def _saved_map_world(grid):
    from sim.mock_world import MockWorld

    class SavedMap(MockWorld):
        def get_pose(self):
            p = super().get_pose()
            return dict(p, frame_id=FRAME) if p.get("usable") else p
    return SavedMap(grid)


def _contacts(grid):
    out, record = [], grid._record

    def counted(event):
        if event.startswith("BLOCKED"):
            out.append(event)
        record(event)
    grid._record = counted
    return out


def survey(store_dir, seed=DET_SEED, nav=NAV):
    """The survey: job survey from the dock, S3's noisy detector, saved to
    `store_dir`. Returns (its row, the map it ended with)."""
    _quiet()
    from control.inventory_store import InventoryStore
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from tests.inventory_sweep import ErringDetector

    grid = build_world(HOUSE)
    contacts = _contacts(grid)
    world = _saved_map_world(grid)
    robot = MockRobot(grid, render=False, exact_verbs=True)
    store = InventoryStore(str(store_dir))
    det = ErringDetector(grid, random.Random(seed), noisy=True)
    runner = MissionRunner(robot, job="survey", policy="explore",
                           navigator=_nav(robot, nav), world=world,
                           mission_time_limit_s=SURVEY_LIMIT_S, detections_fn=det,
                           clock=lambda: grid.sim_time, idle=robot.pass_time,
                           return_to_dock=False)
    runner.inventory_prior = store.latest_for
    runner.inventory_sink = lambda report: store.save("survey", report)
    runner.start()
    ticks = 0
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
    s = runner.status()
    rep = runner.inventory.report()
    return {"mission": "survey", "outcome": s["outcome"], "search_s": round(grid.sim_time, 1),
            "ticks": ticks, "capped": ticks >= TICK_CAP,
            "contacts": sum(c.startswith("BLOCKED at") for c in contacts),
            "reported": [{"label": r["label"], "x_m": r["x_m"], "y_m": r["y_m"],
                          "belief": r["belief"]} for r in rep["reported"]]}, list(world._cells)


def targets(reported):
    """Small objects the survey reported that the found rule names alone
    (3.175's `targets()`), in name order."""
    from sim.maps import build_world
    from sim.maps import complex_house as house
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "memory_moved_3175", ROOT / "evaluations" / "memory-moved-3175" / "sweep.py")
    moved = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(moved)
    alone = set(moved.targets(build_world(HOUSE), HOUSE))
    have = {r["label"] for r in reported}
    return sorted(n for n in house.SMALL_NAMES if n in alone and n in have)


def plan(reported):
    """The 20 (start, target, detector seed), the same in both arms."""
    from tests.inventory_sweep import starts
    names = targets(reported)
    out = []
    for k, ((cx, cy), theta, room) in enumerate(starts(MISSIONS, seed=START_SEED)):
        out.append({"k": k, "start": [cx, cy], "theta": theta, "room": room,
                    "target": names[k % len(names)], "seed": DET_SEED + 1 + k})
    return out


def find(m, arm, store_dir, cells, reported, nav=NAV):
    """One find. `store_dir` already holds the survey's report (surveyed) or
    nothing; `cells` the survey's end map or None."""
    _quiet()
    from control.inventory_store import InventoryStore
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from tests.inventory_sweep import ErringDetector

    grid = build_world(HOUSE)
    grid.x, grid.y, grid.theta = m["start"][0] + 0.5, m["start"][1] + 0.5, m["theta"]
    contacts = _contacts(grid)
    world = _saved_map_world(grid)
    if cells is not None:
        world._cells = list(cells)
    robot = MockRobot(grid, render=False, exact_verbs=True)
    store = InventoryStore(str(store_dir))
    det = ErringDetector(grid, random.Random(m["seed"]), noisy=True)
    runner = MissionRunner(robot, policy="explore", navigator=_nav(robot, nav),
                           world=world, target_object=m["target"], detections_fn=det,
                           mission_time_limit_s=LIMIT_S, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=False)
    runner.inventory_prior = store.latest_for
    runner.inventory_sink = lambda report: store.save(f"find-{m['k']}", report)
    runner.start()
    agent, events = runner.agent, []
    note = agent._event

    def kept(name, pose=None, **fields):
        events.append({"event": name, **fields})
        note(name, pose, **fields)
    agent._event = kept
    path, last = 0.0, (grid.x, grid.y)
    ticks = 0
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
        path += math.dist(last, (grid.x, grid.y))
        last = (grid.x, grid.y)
    s = runner.status()
    goals = [e for e in events if e["event"] == "goal_sent"]
    # The first recall action: a recall goal, or a recall faced where it stands.
    acts = [e for e in events
            if (e["event"] == "goal_sent") or (e["event"] == "recall" and e.get("here"))]
    first = acts[0] if acts else None
    first_kind = None if first is None else ("recall" if first["event"] == "recall"
                                              or first.get("kind") == "recall" else first.get("kind"))
    first_at = None if first is None else first.get("at")
    lms = [(r["x_m"], r["y_m"]) for r in reported if r["label"] == m["target"]]
    first_ok = (first_kind == "recall" and first_at is not None
                and any(math.dist(first_at, p) <= RECALL_AT_M for p in lms))
    tx, ty = next(c for c, n in grid.objects.items() if n == m["target"])
    target_m = math.dist((grid.x, grid.y), (tx + 0.5, ty + 0.5)) * CELL
    return {"arm": arm, "k": m["k"], "target": m["target"], "start_room": m["room"],
            "outcome": s["outcome"], "end_reason": (s.get("end_reason") or "")[:160],
            "search_s": round(grid.sim_time, 1), "ticks": ticks, "capped": ticks >= TICK_CAP,
            "steps": len(runner.memory.actions), "distance_m": round(path * CELL, 2),
            "target_m": round(target_m, 2),
            "wrong_found": s["outcome"] == "found" and target_m > FOUND_M,
            "first_act": first_kind, "first_at": first_at, "first_recall_ok": first_ok,
            "goals": len(goals), "recall_goals": sum(g.get("kind") == "recall" for g in goals),
            "contacts": sum(c.startswith("BLOCKED at") for c in contacts),
            "blocked_turning": sum(c.startswith("BLOCKED turning") for c in contacts)}


def _job(args):
    m, arm, work, cells, reported, nav = args
    d = Path(work) / f"{arm}-{m['k']}"
    if d.exists():
        shutil.rmtree(d)
    if arm == "surveyed":
        shutil.copytree(Path(work) / "survey", d)
    else:
        d.mkdir(parents=True)
    return find(m, arm, d, cells if arm == "surveyed" else None, reported, nav)


def run(out, work, workers, nav=NAV):
    import multiprocessing as mp
    work = Path(work)
    sdir = work / "survey"
    if sdir.exists():
        shutil.rmtree(sdir)
    sdir.mkdir(parents=True)
    row, cells = survey(sdir, nav=nav)
    print(json.dumps({k: v for k, v in row.items() if k != "reported"}), flush=True)
    plans = plan(row["reported"])
    with open(out, "w") as f:
        f.write(json.dumps({**row, "plan": plans, "nav": nav}) + "\n")
        jobs = [(m, arm, str(work), cells, row["reported"], nav) for m in plans for arm in ARMS]
        with mp.Pool(workers) as pool:
            for r in pool.imap_unordered(_job, jobs):
                f.write(json.dumps(r) + "\n")
                f.flush()
                print(r["arm"], r["k"], r["target"], r["outcome"], r["search_s"],
                      r["first_act"], r["first_recall_ok"], flush=True)


def _censored(r):
    return r["search_s"] if r["outcome"] == "found" else LIMIT_S


def score(path):
    rows = [json.loads(line) for line in open(path)]
    sv = rows[0]
    arms = {a: sorted((r for r in rows[1:] if r.get("arm") == a), key=lambda r: r["k"]) for a in ARMS}
    s, u = arms["surveyed"], arms["unsurveyed"]
    found = {a: sum(r["outcome"] == "found" for r in arms[a]) for a in ARMS}
    med = {a: statistics.median(_censored(r) for r in arms[a]) if arms[a] else None for a in ARMS}
    out = {"survey": {k: sv[k] for k in ("outcome", "search_s", "contacts", "capped")},
           "survey_reported": len(sv["reported"]),
           "targets": sorted({m["target"] for m in sv["plan"]}),
           "missions": {a: len(arms[a]) for a in ARMS}}
    out["found"] = {**found, "bar": 19, "met": len(s) == MISSIONS and found["surveyed"] >= 19}
    first = sum(r["first_recall_ok"] for r in s)
    out["first_goal"] = {"recall_first": first, "of": len(s),
                         "missed": [[r["k"], r["target"], r["first_act"]] for r in s if not r["first_recall_ok"]],
                         "met": len(s) == MISSIONS and first == len(s)}
    out["median_s"] = {**med, "ratio": (med["surveyed"] / med["unsurveyed"]) if med["unsurveyed"] else None,
                       "met": bool(s and u) and med["surveyed"] <= 0.5 * med["unsurveyed"]}
    out["nav"] = sv.get("nav", "inflated")
    out["beside"] = {a: {"median_steps": statistics.median(r["steps"] for r in arms[a]),
                         "median_goals": statistics.median(r["goals"] for r in arms[a]),
                         "median_s": med[a],
                         "median_distance_m": statistics.median(r["distance_m"] for r in arms[a]),
                         "median_found_s": (statistics.median(r["search_s"] for r in arms[a]
                                                              if r["outcome"] == "found")
                                            if any(r["outcome"] == "found" for r in arms[a]) else None)}
                     for a in ARMS if arms[a]}
    every = rows[1:]
    out["stability"] = {"wrong_found": sum(r["wrong_found"] for r in every),
                        "contacts": sum(r["contacts"] for r in every) + sv["contacts"],
                        "capped": sum(r["capped"] for r in every) + sv["capped"],
                        "met": not any(r["wrong_found"] or r["contacts"] for r in every)
                        and not sv["contacts"]}
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "run":
        run(a[1], a[2], int(a[3]) if len(a) > 3 else 4, a[4] if len(a) > 4 else NAV)
    elif a and a[0] == "score":
        score(a[1])
    else:
        sys.exit(__doc__)
