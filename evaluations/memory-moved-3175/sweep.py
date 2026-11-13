# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/memory-moved-3175/sweep.py -- 3.175.A: a find whose target moved
between missions on a saved map, in process (`tests/dock_sweep.py`'s way: the
simulator's clock, `InflatedNav`, the frame's detections). The criteria A1-A7
are in `docs/plans/ros-alignment/3.175-memory-when-things-move.md`.

Each trial is one target, one move (another room, within its room, or gone)
and five missions on a real chain: an `InventoryStore` directory of its own and
one saved map's `frame_id` on every pose, so a mission seeds from the head of
the chain and saves its report as the live brain does. Since 3.175.B (amendment
B0) each mission of the chain also starts from the map the previous one ended
with, as the car reloads its saved map; `seed` and `cold` start from nothing.

    python tools/desktop_lock.py shared --why "3.175.A sweep" -- \\
        python evaluations/memory-moved-3175/sweep.py run ROWS.jsonl WORKDIR [WORKERS]
    python evaluations/memory-moved-3175/sweep.py score ROWS.jsonl
"""

import json
import math
import random
import statistics
import sys
import zlib
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

HOUSES = ("complex_house", "scaled_house", "home_first_floor")
ONLY = {"scaled_house": ("red backpack",), "home_first_floor": ("red backpack",)}
FRAME = "map-3175"
LIMIT_S = 1200.0
CELL = 0.30
FOUND_M = 0.9              # 3.80.G2: a find within this of the target is right
SAME_ROOM_MIN = 5          # cells: a move within the room is at least 1.5 m
RECALL_NEAR_M = 1.0        # A3: a recall this close to the old place is to it
TICK_CAP = 20000
MISSIONS = ("seed", "fresh", "stale", "after", "cold")


def targets(grid, house):
    """One-cell objects whose name no other label contains (the found rule's
    substring match would take that other object for the target)."""
    labels = list(grid.objects.values())
    one = [n for n in set(labels) if labels.count(n) == 1]
    ok = [n for n in one if not any(n in other and other != n for other in set(labels))]
    return sorted(n for n in ok if n in ONLY.get(house, ok))


def floor_from(grid, start):
    """True floor reachable from `start`, past nothing solid."""
    from sim.grid_world import CELL_WALL
    seen, q = {start}, deque([start])
    while q:
        c = q.popleft()
        for n in ((c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)):
            if n not in seen and grid._cell(*n) != CELL_WALL and n not in grid.objects:
                seen.add(n)
                q.append(n)
    return seen


def open_cells(grid, floor):
    """Floor whose eight neighbours are floor too: an object put there blocks
    no passage, and `move_object` takes it (not by the robot)."""
    dock = (grid.robot_x, grid.robot_y)
    return sorted(c for c in floor
                  if all((c[0] + dx, c[1] + dy) in floor for dx in (-1, 0, 1) for dy in (-1, 0, 1))
                  and math.dist(c, dock) > 3)


def trials():
    from sim.maps import build_world
    out = []
    for house in HOUSES:
        grid = build_world(house)
        floor = floor_from(grid, (grid.robot_x, grid.robot_y))
        cells = open_cells(grid, floor)
        for t in targets(grid, house):
            p0 = next(c for c, n in grid.objects.items() if n == t)
            room = grid.room_at(*p0)
            rng = random.Random(zlib.crc32(f"{house}:{t}".encode()))
            by_room = {}
            for c in cells:
                by_room.setdefault(grid.room_at(*c), []).append(c)
            others = sorted(r for r in by_room if r not in (room, "unknown"))
            for r in rng.sample(others, min(2, len(others))):
                out.append({"house": house, "target": t, "p0": p0, "move": "room",
                            "p1": rng.choice(by_room[r]), "to_room": r})
            same = [c for c in by_room.get(room, ()) if math.dist(c, p0) >= SAME_ROOM_MIN]
            if same:
                out.append({"house": house, "target": t, "p0": p0, "move": "within",
                            "p1": rng.choice(same), "to_room": room})
            out.append({"house": house, "target": t, "p0": p0, "move": "gone", "p1": None,
                        "to_room": None})
    for i, tr in enumerate(out):
        tr["id"] = f"{tr['house']}:{tr['target']}:{tr['move']}:{i}"
    return out


def mission(trial, where, store_dir, name, cells=None):
    """One find from the house's dock with the target at `where` (a cell, or
    None: gone), on the chain in `store_dir`, starting from the map `cells`
    (3.175.B amendment B0: the previous mission's; None, unknown). Returns
    (the row, the map the mission ended with)."""
    import logging
    logging.disable(logging.WARNING)
    from control.inventory_store import InventoryStore
    from control.mission_runner import MissionRunner
    from sim.maps import build_world
    from sim.mock_robot import MockRobot
    from sim.mock_world import MockWorld
    from tests.dock_sweep import inflated_nav_class

    class SavedMap(MockWorld):
        def get_pose(self):
            p = super().get_pose()
            return dict(p, frame_id=FRAME) if p.get("usable") else p

    grid = build_world(trial["house"])
    p0 = tuple(trial["p0"])
    if where is None:
        objs = dict(grid.objects)
        objs.pop(p0)
        grid.objects = objs
    elif tuple(where) != p0:
        grid.move_object(p0, tuple(where))
    contacts = []
    record = grid._record

    def counted(event):
        if event.startswith("BLOCKED"):
            contacts.append(event)
        record(event)
    grid._record = counted
    world = SavedMap(grid)
    if cells is not None:
        world._cells = list(cells)
    robot = MockRobot(grid, render=False, exact_verbs=True)
    store = InventoryStore(str(store_dir))
    runner = MissionRunner(robot, policy="explore", navigator=inflated_nav_class()(robot),
                           world=world, target_object=trial["target"],
                           mission_time_limit_s=LIMIT_S, clock=lambda: grid.sim_time,
                           idle=robot.pass_time, return_to_dock=False)
    runner.inventory_prior = store.latest_for
    runner.inventory_sink = lambda report: store.save(f"{name}-{trial['id']}".replace(":", "_").replace(" ", "_"), report)
    runner.start()
    agent, events = runner.agent, []
    note = agent._event

    def kept(name, pose=None, **fields):
        # Explore keeps its last MAX_EVENTS only; this list keeps them all,
        # and names the carried place each recall goal was sent for.
        rec = {"event": name, **fields}
        if name == "goal_sent" and fields.get("kind") == "recall" and agent.recall:
            rec["place"] = list(agent.recall[0])
        events.append(rec)
        note(name, pose, **fields)
    agent._event = kept
    ticks = 0
    while runner.tick() and ticks < TICK_CAP:
        ticks += 1
    s = runner.status()
    goals = [e for e in events if e["event"] == "goal_sent"]
    queued = [p for e in events if e["event"] == "recall_set" for p in e.get("places") or []]
    # A recall acted on: a goal sent for a carried place, or a place close
    # enough to be faced where the robot stood (`here`).
    acted = [e["place"] for e in goals if e.get("kind") == "recall" and "place" in e]
    acted += [e["at"] for e in events if e["event"] == "recall" and e.get("here")]
    target_m = None
    if where is not None:
        target_m = math.dist((grid.x, grid.y), (where[0] + 0.5, where[1] + 0.5)) * CELL
    old = ((p0[0] + 0.5) * CELL, (p0[1] + 0.5) * CELL)
    return {"carried_map": cells is not None, "trial": trial["id"], "house": trial["house"], "target": trial["target"],
            "move": trial["move"], "mission": name, "outcome": s["outcome"],
            "end_reason": (s.get("end_reason") or "")[:160], "search_s": round(grid.sim_time, 1),
            "ticks": ticks, "capped": ticks >= TICK_CAP,
            "target_m": None if target_m is None else round(target_m, 2),
            "wrong_found": s["outcome"] == "found" and (target_m is None or target_m > FOUND_M),
            "first_goal": goals[0]["kind"] if goals else None,
            "recall_goals": sum(g["kind"] == "recall" for g in goals),
            "queued": queued, "acted": acted,
            "recalled_old": any(math.dist(p, old) <= RECALL_NEAR_M for p in acted),
            "contacts": sum(c.startswith("BLOCKED at") for c in contacts),
            "blocked_turning": sum(c.startswith("BLOCKED turning") for c in contacts),
            "goals": len(goals)}, list(world._cells)


def run_trial(args):
    trial, work = args
    d = Path(work) / trial["id"].replace(":", "_").replace(" ", "_")
    chain, cold = d / "chain", d / "cold"
    for p in (chain, cold):
        p.mkdir(parents=True, exist_ok=True)
        for f in p.glob("*.json"):
            f.unlink()                    # a re-run trial starts from an empty store
    p0, p1 = trial["p0"], trial["p1"]
    rows, cells = [], None
    for name, where in (("seed", p0), ("fresh", p0), ("stale", p1), ("after", p1)):
        row, cells = mission(trial, where, chain, name, cells)    # B0: the map carried
        rows.append(row)
    if trial["move"] != "gone":
        rows.append(mission(trial, p1, cold, "cold")[0])
    return rows


def run(out, work, workers):
    import multiprocessing as mp
    done = set()
    if Path(out).exists():
        rows = [json.loads(line) for line in open(out)]
        have = {}
        for r in rows:
            have.setdefault(r["trial"], set()).add(r["mission"])
        done = {t for t, ms in have.items()
                if {"seed", "fresh", "stale", "after"} <= ms and ("cold" in ms or ":gone:" in t)}
    todo = [(t, work) for t in trials() if t["id"] not in done]
    print(f"{len(todo)} trials to run", flush=True)
    with mp.Pool(workers) as pool, open(out, "a") as f:
        for rows in pool.imap_unordered(run_trial, todo):
            for r in rows:
                f.write(json.dumps(r) + "\n")
            f.flush()
            print(rows[0]["trial"], [(r["mission"], r["outcome"], r["search_s"]) for r in rows], flush=True)


def _pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(math.ceil(q * len(xs))) - 1)] if xs else None


def score(path):
    rows = [json.loads(line) for line in open(path)]
    by = {}
    for r in rows:
        by.setdefault(r["trial"], {})[r["mission"]] = r      # a re-run trial: its last rows
    full = {k: v for k, v in by.items()
            if {"seed", "fresh", "stale", "after"} <= set(v) and ("cold" in v or ":gone:" in k)}
    moved = [v for v in full.values() if v["stale"]["move"] != "gone" and "cold" in v]
    gone = [v for v in full.values() if v["stale"]["move"] == "gone"]
    f = lambda vs, m: sum(v[m]["outcome"] == "found" for v in vs)  # noqa: E731
    med = lambda vs, m: statistics.median(v[m]["search_s"] for v in vs) if vs else None  # noqa: E731
    out = {"trials": len(full), "moved": len(moved), "gone": len(gone)}
    out["A1"] = {"stale_found": f(moved, "stale"), "cold_found": f(moved, "cold"),
                 "met": f(moved, "stale") >= f(moved, "cold") - 1}
    cost = [v["stale"]["search_s"] - v["cold"]["search_s"] for v in moved]
    out["A2"] = {"median_s": statistics.median(cost) if cost else None, "p90_s": _pct(cost, 0.9),
                 "met": bool(cost) and statistics.median(cost) <= 60 and _pct(cost, 0.9) <= 180}
    again = [v["after"]["trial"] for v in full.values() if v["after"]["recalled_old"]]
    out["A3"] = {"recalled_old_again": len(again), "of": len(full), "trials": again, "met": not again}
    out["A4"] = {"after_found": f(moved, "after"), "cold_found": f(moved, "cold"),
                 "after_median_s": med(moved, "after"), "cold_median_s": med(moved, "cold"),
                 "met": bool(moved) and f(moved, "after") >= f(moved, "cold")
                 and med(moved, "after") <= med(moved, "cold")}
    seeded = [v for v in full.values() if v["seed"]["outcome"] == "found"]
    first = sum(v["fresh"]["first_goal"] == "recall" for v in seeded)
    vs = list(full.values())
    out["A5"] = {"fresh_found": f(vs, "fresh"), "seed_found": f(vs, "seed"),
                 "first_recall": first, "of_seed_found": len(seeded),
                 "fresh_median_s": med(vs, "fresh"), "seed_median_s": med(vs, "seed"),
                 "met": bool(vs) and f(vs, "fresh") >= f(vs, "seed")
                 and first >= 0.95 * len(seeded) and med(vs, "fresh") <= 0.5 * med(vs, "seed")}
    out["A6"] = {"found_when_gone": sum(v[m]["outcome"] == "found" for v in gone for m in ("stale", "after")),
                 "met": all(v[m]["outcome"] != "found" for v in gone for m in ("stale", "after"))}
    out["A7"] = {"wrong_found": sum(r["wrong_found"] for r in rows),
                 "contacts": sum(r["contacts"] for r in rows), "capped": sum(r["capped"] for r in rows),
                 "met": not any(r["wrong_found"] or r["contacts"] for r in rows)}
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "run":
        run(a[1], a[2], int(a[3]) if len(a) > 3 else 4)
    elif a and a[0] == "score":
        score(a[1])
    elif a and a[0] == "trials":
        for t in trials():
            print(t["id"], t["p0"], t["p1"], t["to_room"])
    else:
        sys.exit(__doc__)
