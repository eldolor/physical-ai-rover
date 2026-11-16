# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.178.B's simulator sweep: does the watch list pin what it passed, and
nothing it did not? (criteria B1, B2 and B4's simulator half, written in
3.178.A before this ran.)

    python evaluations/watch-list-3178/sweep.py            # both sweeps -> results.json
    python evaluations/watch-list-3178/sweep.py --missions 2 --out /tmp/x.json

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
from brain.watch_list import DEFAULT_WATCH_LIST, item_for  # noqa: E402
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
SEED = 3178
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
    return {"start_room": room, "outcome": runner.status()["outcome"],
            "actions": [a.action for a in runner.memory.actions],
            "watch": report.get("watch"), "nearest": det.nearest,
            "injected": det.injected, "wall_s": round(time.perf_counter() - t0, 2)}


def _dist(item: str, x: float, y: float) -> float:
    return min((base._dist_to_instance(x, y, k) for k in ITEM_INSTANCES[item]), default=math.inf)


def score_mission(r: dict) -> dict:
    passes, found, found_loose, wrong = [], [], [], []
    pins = (r["watch"] or {}).get("pins", {})
    for item, rows in pins.items():
        for p in rows:
            d = _dist(item, p["x_m"], p["y_m"])
            if d > PIN_M:
                wrong.append({"item": item, "x_m": p["x_m"], "y_m": p["y_m"],
                              "belief": p["belief"], "to_instance_m": None if math.isinf(d) else round(d, 3)})
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
    return {"passes": passes, "found": found, "found_loose": found_loose, "wrong_pins": wrong}


def sweep(n: int, target: str, seed: int, max_steps: int, log=sys.stderr) -> dict:
    starts = base.starts(n=n, seed=seed)
    rows = []
    for k, start in enumerate(starts):
        on = run_one(start, list_on=True, seed=seed * 100 + k, target=target, max_steps=max_steps)
        off = run_one(start, list_on=False, seed=seed * 100 + k, target=target, max_steps=max_steps)
        s = score_mission(on)
        same = on["outcome"] == off["outcome"] and on["actions"] == off["actions"]
        rows.append({"mission": k + 1, "start_room": on["start_room"],
                     "outcome_on": on["outcome"], "outcome_off": off["outcome"],
                     "steps": len(on["actions"]), "same_as_off": same,
                     "injected": on["injected"],
                     "nearest_m": {HOUSE.INSTANCES[i][0]: d for i, d in sorted(on["nearest"].items())},
                     "pins": {item: [(p["x_m"], p["y_m"], p["belief"]) for p in ps]
                              for item, ps in (on["watch"] or {}).get("pins", {}).items() if ps},
                     "passes": [HOUSE.INSTANCES[i][0] for i in s["passes"]],
                     "found": [HOUSE.INSTANCES[i][0] for i in s["found"]],
                     "found_1m": [HOUSE.INSTANCES[i][0] for i in s["found_loose"]],
                     "wrong_pins": s["wrong_pins"]})
        print(f"{target:<12} mission {k + 1:2d} {on['start_room']:<12} {on['outcome']:<11} "
              f"passes {len(s['passes'])} found {len(s['found'])} wrong {len(s['wrong_pins'])} "
              f"same {same}  {on['wall_s'] + off['wall_s']:.0f}s", file=log, flush=True)
    passes = sum(len(r["passes"]) for r in rows)
    found = sum(len(r["found"]) for r in rows)
    found_1m = sum(len(r["found_1m"]) for r in rows)
    wrong = sum(len(r["wrong_pins"]) for r in rows)
    arrived = lambda key: sum(1 for r in rows if r[key] in ("found", "arrived_unconfirmed"))  # noqa: E731
    return {"target": target, "missions": len(rows), "max_steps": max_steps, "seed": seed,
            "passes": passes, "found": found, "found_rate": found / passes if passes else None,
            "found_1m": found_1m, "wrong_pins": wrong,
            "wrong_pins_per_mission": wrong / len(rows) if rows else None,
            "same_as_off": sum(r["same_as_off"] for r in rows),
            "arrived_on": arrived("outcome_on"), "arrived_off": arrived("outcome_off"),
            "rows": rows}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--missions", type=int, default=MISSIONS)
    ap.add_argument("--steps", type=int, default=MAX_STEPS)
    ap.add_argument("--out", default=str(Path(__file__).with_name("results.json")))
    args = ap.parse_args(argv)
    out = {"list": list(LIST), "list_false_rate": LIST_FALSE_RATE, "pass_range_m": PASS_RANGE_M,
           "pin_m": PIN_M, "noise": {"bearing_sigma_deg": base.BEARING_SIGMA_DEG,
                                     "miss_rate": base.MISS_RATE, "relabel_rate": base.RELABEL_RATE},
           "search": sweep(args.missions, base.ABSENT_TARGET, SEED, args.steps),
           "present": sweep(args.missions, PRESENT_TARGET, SEED + 1, args.steps)}
    Path(args.out).write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    for key in ("search", "present"):
        s = out[key]
        print(f"{key}: passes {s['passes']} found {s['found']} ({s['found_rate']}) "
              f"found@1m {s['found_1m']} wrong {s['wrong_pins']} same {s['same_as_off']}/{s['missions']} "
              f"arrived on {s['arrived_on']} off {s['arrived_off']}", file=sys.stderr)


if __name__ == "__main__":
    main()
