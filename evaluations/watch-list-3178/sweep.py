# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.B's simulator sweep: does the watch list pin what it passed, and
nothing it did not? (criteria B1, B2 and B4's simulator half, written in
3.178.A before this ran.)

    python evaluations/watch-list-3178/sweep.py            # 3.178.D's fresh seeds -> results-d.json
    python evaluations/watch-list-3178/sweep.py --seed 3178 --out results-d-bseed.json   # B's seeds
    python evaluations/watch-list-3178/sweep.py --missions 2 --out /tmp/x.json
    python evaluations/watch-list-3178/sweep.py --placed placements.json --steps STEPS \
        --seed SEED --out results-e.json             # 3.178.E (its section gives the numbers)
    python evaluations/watch-list-3178/sweep.py --placed placements.json --steps 150 \
        --seed 6178 --workers 4 --out results-f.json # 3.178.F's fresh run

`--placed` (3.178.E) adds the list items `placement.py` chose to the house
before each mission (solid, one cell each, as the house's own small things),
and the starts then avoid them (`starts`). Without it the house is the
unchanged `complex_house` and the starts are `inventory_sweep.starts`, so
B's and D's numbers reproduce (the rows gain `reached` and `carried`).

On a shared desktop, run it under the heavy-run lock (CLAUDE.md 3c):
`python tools/desktop_lock.py shared --why watch-list -- python ...`.

Each report is scored under both pin rules (`RULES`, 3.178.D), paired.
`results.json` is 3.178.B's record in B's schema (top-level `found`,
`wrong_pins`), written by this script as of `c4849877`; this version
writes the per-rule schema (`rules.label`, `rules.votes`), so do not
regenerate `results.json` with it.

Every mission runs the WHOLE path: `MissionRunner` -> the frontier policy ->
`robot/safety.py` -> `MockRobot` in `complex_house`. The inventory is fed by
the runner's own frame hook through 3.46's erring detector
(`tests/inventory_sweep.ErringDetector`: 3 deg bearing sigma, 10% misses,
5% of frames with one wrong-class label, never a list item), and with the
list ON also:

* on `LIST_FALSE_RATE` of frames, one detection whose label is drawn
  uniformly from the list, at a uniform bearing in the field (3.178.A's
  list-labelled false detection), from its OWN seeded generator, so the
  3.46 noise is draw for draw the same with the list on and off.

The pins are the runner's own report's `watch` block (`brain.watch_list`).

Scored:

* B1, a PASS: a list item instance in the house (keys; the TV remote) that a
  TRUE detection of the sim (field, occlusion and range from the renderer,
  so a clear line of sight) reported within `PASS_RANGE_M` (2.5 m), once
  per mission and instance. FOUND: a pin of its item within `PIN_M` (0.5 m,
  the section's pin-is-right distance) of the instance; also reported at
  1.0 m (3.46's match distance), which is not the bar.
* B2, a WRONG PIN: a pin with no instance of its item within `PIN_M`.
* 3.178.F, a pass's VIEWS: how many independent viewpoints its true
  sightings came from, by the inventory's own rule (`inventory._new_view`:
  `NEW_VIEW_M` moved or `NEW_VIEW_DEG` turned, greedily, as
  `inventory_sweep._views` counts them), over every true sighting of the
  instance out to the inventory's own placement horizon (`MAX_RANGE_M`,
  4 m; the pass itself still needs one within 2.5 m). A report needs two independent
  hits (`REPORT_BELIEF`), and frames from one viewpoint are one look by
  design, so a pass seen from fewer than `MIN_VIEWS` (2) can never reach a
  report under any rule. `passes_2v` / `reached_2v` / `found_2v` count over
  the passes with at least two; B1's `passes` (every pass) are unchanged.
* B4 (sim half): the same starts with the list off; outcome and every
  action must be equal mission by mission. The search sweep has an absent
  target (it explores to `max_steps`, as 3.46's did); a second sweep with a
  target in the house gives the arrival rate.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from brain import inventory as inv  # noqa: E402
from brain.watch_list import DEFAULT_WATCH_LIST, item_for, watch_block  # noqa: E402
from control.mission_runner import MissionRunner  # noqa: E402
from sim.maps import build_world  # noqa: E402
from sim.mock_robot import MockRobot  # noqa: E402
from tests import inventory_sweep as base  # noqa: E402
from tests.conftest import mock_world_for  # noqa: E402

LIST = DEFAULT_WATCH_LIST
LIST_FALSE_RATE = 0.05
PASS_RANGE_M = 2.5
PIN_M = 0.5
LOOSE_M = 1.0
MISSIONS = 20
MAX_STEPS = base.MAX_STEPS          # 150, 3.46's
SEED = 3178                 # 3.178.B's (search; present SEED + 1)
FRESH_SEED = 4178           # 3.178.D's, fresh (written before its run)
RULES = ("label", "votes")  # 3.178.A's pin rule; 3.178.D's
# 3.178.F: the views a report needs -- two independent hits reach
# REPORT_BELIEF (one hit reads 0.70), and one viewpoint is one look.
MIN_VIEWS = 2
PRESENT_TARGET = "red backpack"
HOUSE = base.house


def instances(placed=()) -> list:
    """The house's instances, then each placed list item (3.178.E) as one
    more: [(name, frozenset(cells))]."""
    return list(HOUSE.INSTANCES) + [(p["item"], frozenset({tuple(p["cell"])})) for p in placed]


def item_instances(insts: list) -> dict:
    """Each list item's instance indices in `insts` (by `item_for`, the
    report's rule)."""
    return {item: [k for k, (name, _) in enumerate(insts) if item_for(name, LIST) == item]
            for item in LIST}


# B's and D's house (no placements).
INSTANCES = instances()
ITEM_INSTANCES = item_instances(INSTANCES)


def dist_to_cells(x: float, y: float, cells) -> float:
    """`inventory_sweep._dist_to_instance`'s measure, over any cells."""
    best = math.inf
    for i, j in cells:
        dx = max(i * base.CELL_M - x, 0.0, x - (i + 1) * base.CELL_M)
        dy = max(j * base.CELL_M - y, 0.0, y - (j + 1) * base.CELL_M)
        best = min(best, math.hypot(dx, dy))
    return best


def starts(n: int, seed: int, placed=()) -> list:
    """`inventory_sweep.starts`, and with placements the same rule (rooms in
    turn; a free cell whose 3 x 3 holds no object) with the placed cells
    counted as objects, so no robot starts on or against one."""
    if not placed:
        return base.starts(n=n, seed=seed)
    objects = set(HOUSE.OBJECTS) | {tuple(p["cell"]) for p in placed}
    rng = random.Random(seed)
    rooms = sorted(r for r in HOUSE.ROOMS if HOUSE.ROOMS[r])
    out = []
    for k in range(n):
        room = rooms[k % len(rooms)]
        cells = sorted(c for c in HOUSE.ROOMS[room]
                       if all((c[0] + dx, c[1] + dy) in HOUSE.ROOMS[room]
                              and (c[0] + dx, c[1] + dy) not in objects
                              for dx in (-1, 0, 1) for dy in (-1, 0, 1)))
        if not cells:
            continue
        out.append((rng.choice(cells), rng.uniform(-math.pi, math.pi), room))
    return out[:n]


class ListErringDetector(base.ErringDetector):
    """3.46's erring detector plus, with the list on, 3.178.A's
    list-labelled false detection. Also records each list instance's
    nearest TRUE detection range this mission (the B1 pass)."""

    def __init__(self, grid, rng, list_rng=None, noisy=True, insts=None,
                 false_rate=LIST_FALSE_RATE):
        super().__init__(grid, rng, noisy=noisy)
        self.list_rng, self.false_rate = list_rng, false_rate
        self.insts = INSTANCES if insts is None else insts
        self.item_insts = item_instances(self.insts)
        self.nearest: dict = {}          # instance -> nearest true range (m)
        self.poses: dict = {}            # 3.178.F: instance -> its true sightings' poses
        self.injected = 0

    def __call__(self, frame):
        g = self.grid
        view = g.theta + g.pan * base.PAN_ANGLE_RAD
        for d in frame.get("detections") or []:
            item = item_for(d.get("label"), LIST)
            # 3.178.F: a view is any true sighting the inventory could place
            # (out to its MAX_RANGE_M); a PASS is one within PASS_RANGE_M.
            if item is None or d["distance_m"] > max(PASS_RANGE_M, inv.MAX_RANGE_M):
                continue
            a = view + math.radians(d["bearing_deg"])
            r = d["distance_m"] / base.CELL_M
            cell = (int(math.floor(g.x + r * math.cos(a))), int(math.floor(g.y + r * math.sin(a))))
            for k in self.item_insts[item]:
                if cell in self.insts[k][1] or _near_cell(cell, self.insts[k][1]):
                    if d["distance_m"] <= PASS_RANGE_M:
                        self.nearest[k] = min(self.nearest.get(k, 9e9), d["distance_m"])
                    poses = self.poses.setdefault(k, [])
                    pose = {"x_m": g.x * base.CELL_M, "y_m": g.y * base.CELL_M,
                            "heading_deg": g.heading_deg}
                    if not poses or poses[-1] != pose:     # once a frame
                        poses.append(pose)
        out = super().__call__(frame)
        if out is not None and self.list_rng is not None and self.list_rng.random() < self.false_rate:
            fov = float(frame.get("fov_deg") or inv.DEFAULT_FOV_DEG)
            out = out + [{"label": self.list_rng.choice(LIST),
                          "bearing_deg": self.list_rng.uniform(-fov / 2, fov / 2),
                          "confidence": 1.0}]
            self.injected += 1
        return out


def views(poses: list) -> int:
    """3.178.F: independent viewpoints among `poses`, by the inventory's own
    new-view rule (`inventory_sweep._views`)."""
    return base._views(poses)


def _near_cell(cell, cells) -> bool:
    return any(abs(cell[0] - c[0]) <= 1 and abs(cell[1] - c[1]) <= 1 for c in cells)


def run_one(start, *, list_on: bool, seed: int, target: str = base.ABSENT_TARGET,
            max_steps: int = MAX_STEPS, placed=(), false_rate: float = LIST_FALSE_RATE) -> dict:
    (cx, cy), theta, room = start
    grid = build_world("complex_house")
    if placed:
        # 3.178.E: the placed items, solid as the house's own small things.
        # `objects` is replaced, never mutated (the grid caches against it).
        grid.objects = {**grid.objects, **{tuple(p["cell"]): p["item"] for p in placed}}
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    det = ListErringDetector(grid, random.Random(seed),
                             list_rng=random.Random(seed * 7 + 1) if list_on else None,
                             insts=instances(placed), false_rate=false_rate)
    runner = MissionRunner(robot, target_object=target, max_steps=max_steps,
                           policy="frontier", world=mock_world_for(robot),
                           detections_fn=det, watch_list=LIST if list_on else ())
    t0 = time.perf_counter()
    runner.start()
    while runner.tick():
        pass
    report = runner.inventory_report()
    # 3.178.D: the one report scored under both pin rules, paired.
    watch = {rule: (watch_block(report, LIST, target, rule=rule) if list_on else None)
             for rule in RULES}
    # The default rule's block IS the runner's own (the shipped path); the
    # other rule is recomputed from the same report.
    assert watch["label"] == report.get("watch"), "the runner's watch block moved"
    return {"start_room": room, "outcome": runner.status()["outcome"],
            "actions": [a.action for a in runner.memory.actions],
            "watch": watch, "nearest": det.nearest,
            # 3.178.F: each pass's independent viewpoints (`views`).
            "views": {k: views(ps) for k, ps in det.poses.items()},
            "reported": [{k: lm.get(k) for k in ("label", "x_m", "y_m", "belief", "carried")}
                         for lm in report.get("reported") or []],   # reported only, by construction
            "carried": sum(1 for lm in report.get("reported") or [] if lm.get("carried")),
            "injected": det.injected, "wall_s": round(time.perf_counter() - t0, 2)}


def _dist(item: str, x: float, y: float, insts=None, ii=None) -> float:
    insts = INSTANCES if insts is None else insts
    ii = item_instances(insts) if ii is None else ii
    return min((dist_to_cells(x, y, insts[k][1]) for k in ii[item]), default=math.inf)


def score_mission(r: dict, rule: str, insts=None) -> dict:
    insts = INSTANCES if insts is None else insts
    ii = item_instances(insts)
    passes, found, found_loose, wrong, true, reached = [], [], [], [], 0, []
    pins = ((r["watch"] or {}).get(rule) or {}).get("pins", {})
    for item, rows in pins.items():
        for p in rows:
            d = _dist(item, p["x_m"], p["y_m"], insts, ii)
            if d > PIN_M:
                wrong.append({"item": item, "x_m": p["x_m"], "y_m": p["y_m"],
                              "belief": p["belief"], "to_instance_m": None if math.isinf(d) else round(d, 3)})
            else:
                true += 1
    for item, ks in ii.items():
        for k in ks:
            if k not in r["nearest"]:
                continue
            passes.append(k)
            ds = [dist_to_cells(p["x_m"], p["y_m"], insts[k][1]) for p in pins.get(item, [])]
            if any(d <= PIN_M for d in ds):
                found.append(k)
            if any(d <= LOOSE_M for d in ds):
                found_loose.append(k)
            # 3.178.E, information only: the pass reached a report -- a
            # reported landmark naming its item within PIN_M, whatever its votes.
            if any(item_for(lm["label"], LIST) == item
                   and dist_to_cells(lm["x_m"], lm["y_m"], insts[k][1]) <= PIN_M
                   for lm in r.get("reported") or []):
                reached.append(k)
    two = [k for k in passes if (r.get("views") or {}).get(k, 0) >= MIN_VIEWS]   # 3.178.F
    return {"passes": passes, "found": found, "found_loose": found_loose, "wrong_pins": wrong,
            "true_pins": true, "reached": reached, "passes_2v": two,
            "reached_2v": [k for k in reached if k in two],
            "found_2v": [k for k in found if k in two]}


def _pair(job) -> tuple:
    """One start, list on and off (same seed): the paired unit of the sweep."""
    k, start, target, seed, max_steps, placed, false_rate = job
    on = run_one(start, list_on=True, seed=seed * 100 + k, target=target, max_steps=max_steps,
                 placed=placed, false_rate=false_rate)
    off = run_one(start, list_on=False, seed=seed * 100 + k, target=target, max_steps=max_steps,
                  placed=placed)
    return on, off


def sweep(n: int, target: str, seed: int, max_steps: int, log=sys.stderr, placed=(),
          workers: int = 1, false_rate: float = LIST_FALSE_RATE) -> dict:
    insts = instances(placed)
    jobs = [(k, start, target, seed, max_steps, placed, false_rate)
            for k, start in enumerate(starts(n, seed, placed))]
    if workers > 1:
        with ProcessPoolExecutor(workers) as ex:
            pairs = list(ex.map(_pair, jobs))     # in order: mission k is row k
    else:
        pairs = [_pair(j) for j in jobs]
    rows = []
    for k, (on, off) in enumerate(pairs):
        sc = {rule: score_mission(on, rule, insts) for rule in RULES}
        same = on["outcome"] == off["outcome"] and on["actions"] == off["actions"]
        name = lambda ks: [insts[i][0] for i in ks]  # noqa: E731
        row = {"mission": k + 1, "start_room": on["start_room"],
               "outcome_on": on["outcome"], "outcome_off": off["outcome"],
               "steps": len(on["actions"]), "same_as_off": same,
               "injected": on["injected"], "carried": on["carried"],
               "nearest_m": {f"{insts[i][0]}#{i}" if placed else insts[i][0]: d
                             for i, d in sorted(on["nearest"].items())},
               "passes": name(sc["label"]["passes"]),
               "reached": name(sc["label"]["reached"]),
               "views": {f"{insts[i][0]}#{i}" if placed else insts[i][0]: on["views"].get(i, 0)
                         for i in sc["label"]["passes"]},
               "passes_2v": name(sc["label"]["passes_2v"]),
               "reached_2v": name(sc["label"]["reached_2v"])}
        for rule, sr in sc.items():
            w = on["watch"][rule] or {}
            row[rule] = {"pins": {item: [(p["x_m"], p["y_m"], p["belief"], p.get("votes")) for p in ps]
                                  for item, ps in w.get("pins", {}).items() if ps},
                         "held": w.get("held", 0),
                         "found": name(sr["found"]), "found_1m": name(sr["found_loose"]),
                         "found_2v": name(sr["found_2v"]),
                         "true_pins": sr["true_pins"], "wrong_pins": sr["wrong_pins"]}
        rows.append(row)
        print(f"{target:<12} mission {k + 1:2d} {on['start_room']:<12} {on['outcome']:<11} "
              f"passes {len(row['passes'])} reached {len(row['reached'])} "
              + " ".join(f"{rule}: found {len(row[rule]['found'])} true {row[rule]['true_pins']} "
                         f"wrong {len(row[rule]['wrong_pins'])}" for rule in RULES)
              + f" same {same}  {on['wall_s'] + off['wall_s']:.0f}s", file=log, flush=True)
    passes = sum(len(r["passes"]) for r in rows)
    reached = sum(len(r["reached"]) for r in rows)
    passes_2v = sum(len(r["passes_2v"]) for r in rows)
    arrived = lambda key: sum(1 for r in rows if r[key] in ("found", "arrived_unconfirmed"))  # noqa: E731
    by_rule = {}
    for rule in RULES:
        found = sum(len(r[rule]["found"]) for r in rows)
        wrong = sum(len(r[rule]["wrong_pins"]) for r in rows)
        found_2v = sum(len(r[rule]["found_2v"]) for r in rows)
        by_rule[rule] = {"found": found, "found_rate": found / passes if passes else None,
                         "found_2v": found_2v,
                         "found_1m": sum(len(r[rule]["found_1m"]) for r in rows),
                         "true_pins": sum(r[rule]["true_pins"] for r in rows),
                         "wrong_pins": wrong,
                         "wrong_pins_per_mission": wrong / len(rows) if rows else None,
                         "held": sum(r[rule]["held"] for r in rows)}
    return {"target": target, "missions": len(rows), "max_steps": max_steps, "seed": seed,
            "passes": passes, "reached": reached, "rules": by_rule,
            "passes_2v": passes_2v, "reached_2v": sum(len(r["reached_2v"]) for r in rows),
            "carried": sum(r["carried"] for r in rows),
            "same_as_off": sum(r["same_as_off"] for r in rows),
            "arrived_on": arrived("outcome_on"), "arrived_off": arrived("outcome_off"),
            "rows": rows}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--missions", type=int, default=MISSIONS)
    ap.add_argument("--steps", type=int, default=MAX_STEPS)
    ap.add_argument("--seed", type=int, default=FRESH_SEED,
                    help="the search sweep's seed; present is seed + 1 (B: 3178, D: 4178)")
    ap.add_argument("--out", default=str(Path(__file__).with_name("results-d.json")))
    ap.add_argument("--placed", default=None,
                    help="3.178.E: a placements.json whose `placed` items join the house")
    ap.add_argument("--list-false-rate", type=float, default=LIST_FALSE_RATE,
                    help="diagnosis on tuning seeds only (0: no list-labelled false detections)")
    ap.add_argument("--workers", type=int, default=1,
                    help="processes; each mission pair is independent, so rows do not change")
    args = ap.parse_args(argv)
    placed = ()
    if args.placed:
        path = Path(args.placed)
        if not path.is_absolute() and not path.exists():
            path = Path(__file__).with_name(args.placed)
        placed = tuple(json.loads(path.read_text())["placed"])
    out = {"list": list(LIST), "list_false_rate": args.list_false_rate, "pass_range_m": PASS_RANGE_M,
           "pin_m": PIN_M, "noise": {"bearing_sigma_deg": base.BEARING_SIGMA_DEG,
                                     "miss_rate": base.MISS_RATE, "relabel_rate": base.RELABEL_RATE},
           "rules": list(RULES),
           "search": sweep(args.missions, base.ABSENT_TARGET, args.seed, args.steps,
                           placed=placed, workers=args.workers,
                           false_rate=args.list_false_rate),
           "present": sweep(args.missions, PRESENT_TARGET, args.seed + 1, args.steps,
                            placed=placed, workers=args.workers,
                            false_rate=args.list_false_rate)}
    if placed:
        out["placed"] = list(placed)
    Path(args.out).write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    for key in ("search", "present"):
        s = out[key]
        print(f"{key}: passes {s['passes']} reached {s['reached']} "
              f"2-view passes {s['passes_2v']} reached {s['reached_2v']} carried {s['carried']} same {s['same_as_off']}/{s['missions']} "
              f"arrived on {s['arrived_on']} off {s['arrived_off']}", file=sys.stderr)
        for rule, v in s["rules"].items():
            print(f"  {rule}: found {v['found']} true {v['true_pins']} wrong {v['wrong_pins']} "
                  f"held {v['held']}", file=sys.stderr)


if __name__ == "__main__":
    main()
