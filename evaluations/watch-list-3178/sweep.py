# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.B's simulator sweep: does the watch list pin what it passed, and
nothing it did not? (criteria B1, B2 and B4's simulator half, written in
3.178.A before this ran.)

    python evaluations/watch-list-3178/sweep.py            # 3.178.D's fresh seeds -> results-d.json
    python evaluations/watch-list-3178/sweep.py --seed 3178 --out results-d-bseed.json   # B's seeds
    python evaluations/watch-list-3178/sweep.py --missions 2 --out /tmp/x.json

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
PRESENT_TARGET = "red backpack"
HOUSE = base.house
# The house's instances of each list item (by `item_for`, the report's rule).
ITEM_INSTANCES = {item: [k for k, (name, _) in enumerate(HOUSE.INSTANCES)
                         if item_for(name, LIST) == item] for item in LIST}


class ListErringDetector(base.ErringDetector):
    """3.46's erring detector plus, with the list on, 3.178.A's
    list-labelled false detection. Also records each list instance's
    nearest TRUE detection range this mission (the B1 pass)."""

    def __init__(self, grid, rng, list_rng=None, noisy=True):
        super().__init__(grid, rng, noisy=noisy)
        self.list_rng = list_rng
        self.nearest: dict = {}          # instance -> nearest true range (m)
        self.injected = 0

    def __call__(self, frame):
        g = self.grid
        view = g.theta + g.pan * base.PAN_ANGLE_RAD
        for d in frame.get("detections") or []:
            item = item_for(d.get("label"), LIST)
            if item is None or d["distance_m"] > PASS_RANGE_M:
                continue
            a = view + math.radians(d["bearing_deg"])
            r = d["distance_m"] / base.CELL_M
            cell = (int(math.floor(g.x + r * math.cos(a))), int(math.floor(g.y + r * math.sin(a))))
            for k in ITEM_INSTANCES[item]:
                if cell in HOUSE.INSTANCES[k][1] or _near_cell(cell, HOUSE.INSTANCES[k][1]):
                    self.nearest[k] = min(self.nearest.get(k, 9e9), d["distance_m"])
        out = super().__call__(frame)
        if out is not None and self.list_rng is not None and self.list_rng.random() < LIST_FALSE_RATE:
            fov = float(frame.get("fov_deg") or inv.DEFAULT_FOV_DEG)
            out = out + [{"label": self.list_rng.choice(LIST),
                          "bearing_deg": self.list_rng.uniform(-fov / 2, fov / 2),
                          "confidence": 1.0}]
            self.injected += 1
        return out


def _near_cell(cell, cells) -> bool:
    return any(abs(cell[0] - c[0]) <= 1 and abs(cell[1] - c[1]) <= 1 for c in cells)


def run_one(start, *, list_on: bool, seed: int, target: str = base.ABSENT_TARGET,
            max_steps: int = MAX_STEPS) -> dict:
    (cx, cy), theta, room = start
    grid = build_world("complex_house")
    grid.x, grid.y, grid.theta = cx + 0.5, cy + 0.5, theta
    robot = MockRobot(grid, render=False)
    det = ListErringDetector(grid, random.Random(seed),
                             list_rng=random.Random(seed * 7 + 1) if list_on else None)
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
            "injected": det.injected, "wall_s": round(time.perf_counter() - t0, 2)}


def _dist(item: str, x: float, y: float) -> float:
    return min((base._dist_to_instance(x, y, k) for k in ITEM_INSTANCES[item]), default=math.inf)


def score_mission(r: dict, rule: str) -> dict:
    passes, found, found_loose, wrong, true = [], [], [], [], 0
    pins = ((r["watch"] or {}).get(rule) or {}).get("pins", {})
    for item, rows in pins.items():
        for p in rows:
            d = _dist(item, p["x_m"], p["y_m"])
            if d > PIN_M:
                wrong.append({"item": item, "x_m": p["x_m"], "y_m": p["y_m"],
                              "belief": p["belief"], "to_instance_m": None if math.isinf(d) else round(d, 3)})
            else:
                true += 1
    for item, ks in ITEM_INSTANCES.items():
        for k in ks:
            if k not in r["nearest"]:
                continue
            passes.append(k)
            ds = [base._dist_to_instance(p["x_m"], p["y_m"], k) for p in pins.get(item, [])]
            if any(d <= PIN_M for d in ds):
                found.append(k)
            if any(d <= LOOSE_M for d in ds):
                found_loose.append(k)
    return {"passes": passes, "found": found, "found_loose": found_loose, "wrong_pins": wrong,
            "true_pins": true}


def sweep(n: int, target: str, seed: int, max_steps: int, log=sys.stderr) -> dict:
    starts = base.starts(n=n, seed=seed)
    rows = []
    for k, start in enumerate(starts):
        on = run_one(start, list_on=True, seed=seed * 100 + k, target=target, max_steps=max_steps)
        off = run_one(start, list_on=False, seed=seed * 100 + k, target=target, max_steps=max_steps)
        sc = {rule: score_mission(on, rule) for rule in RULES}
        same = on["outcome"] == off["outcome"] and on["actions"] == off["actions"]
        name = lambda ks: [HOUSE.INSTANCES[i][0] for i in ks]  # noqa: E731
        row = {"mission": k + 1, "start_room": on["start_room"],
               "outcome_on": on["outcome"], "outcome_off": off["outcome"],
               "steps": len(on["actions"]), "same_as_off": same,
               "injected": on["injected"],
               "nearest_m": {HOUSE.INSTANCES[i][0]: d for i, d in sorted(on["nearest"].items())},
               "passes": name(sc["label"]["passes"])}
        for rule, sr in sc.items():
            w = on["watch"][rule] or {}
            row[rule] = {"pins": {item: [(p["x_m"], p["y_m"], p["belief"], p.get("votes")) for p in ps]
                                  for item, ps in w.get("pins", {}).items() if ps},
                         "held": w.get("held", 0),
                         "found": name(sr["found"]), "found_1m": name(sr["found_loose"]),
                         "true_pins": sr["true_pins"], "wrong_pins": sr["wrong_pins"]}
        rows.append(row)
        print(f"{target:<12} mission {k + 1:2d} {on['start_room']:<12} {on['outcome']:<11} "
              f"passes {len(row['passes'])} "
              + " ".join(f"{rule}: found {len(row[rule]['found'])} true {row[rule]['true_pins']} "
                         f"wrong {len(row[rule]['wrong_pins'])}" for rule in RULES)
              + f" same {same}  {on['wall_s'] + off['wall_s']:.0f}s", file=log, flush=True)
    passes = sum(len(r["passes"]) for r in rows)
    arrived = lambda key: sum(1 for r in rows if r[key] in ("found", "arrived_unconfirmed"))  # noqa: E731
    by_rule = {}
    for rule in RULES:
        found = sum(len(r[rule]["found"]) for r in rows)
        wrong = sum(len(r[rule]["wrong_pins"]) for r in rows)
        by_rule[rule] = {"found": found, "found_rate": found / passes if passes else None,
                         "found_1m": sum(len(r[rule]["found_1m"]) for r in rows),
                         "true_pins": sum(r[rule]["true_pins"] for r in rows),
                         "wrong_pins": wrong,
                         "wrong_pins_per_mission": wrong / len(rows) if rows else None,
                         "held": sum(r[rule]["held"] for r in rows)}
    return {"target": target, "missions": len(rows), "max_steps": max_steps, "seed": seed,
            "passes": passes, "rules": by_rule,
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
    args = ap.parse_args(argv)
    out = {"list": list(LIST), "list_false_rate": LIST_FALSE_RATE, "pass_range_m": PASS_RANGE_M,
           "pin_m": PIN_M, "noise": {"bearing_sigma_deg": base.BEARING_SIGMA_DEG,
                                     "miss_rate": base.MISS_RATE, "relabel_rate": base.RELABEL_RATE},
           "rules": list(RULES),
           "search": sweep(args.missions, base.ABSENT_TARGET, args.seed, args.steps),
           "present": sweep(args.missions, PRESENT_TARGET, args.seed + 1, args.steps)}
    Path(args.out).write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    for key in ("search", "present"):
        s = out[key]
        print(f"{key}: passes {s['passes']} same {s['same_as_off']}/{s['missions']} "
              f"arrived on {s['arrived_on']} off {s['arrived_off']}", file=sys.stderr)
        for rule, v in s["rules"].items():
            print(f"  {rule}: found {v['found']} true {v['true_pins']} wrong {v['wrong_pins']} "
                  f"held {v['held']}", file=sys.stderr)


if __name__ == "__main__":
    main()
