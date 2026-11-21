# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.199 G2-G3: a person's goals beside a no-go zone, against the bridge's
drawn mask and NavFn's goal tolerance, in process.

Per goal (seeded): an axis-aligned zone in the house frame (sides 0.20-3.0 m,
one in four a 0.10 m wall), a rigid anchor into ROS's map frame (a uniform
rotation, a y flip half the time -- the house frame's north is ROS's -y --
and a translation up to 50 m), a uniform lattice phase; the zone drawn by the
bridge's own code (`keepout.with_margin`, `keepout.mask`); a goal at a
uniform distance d outside the zone, a uniform point along the offset curve
(an edge or a corner's arc).

The robot server's answer is `world/nogo.goal_zone` (3.199's D5) or, for
`dev`'s rule, `world/nogo.zone_at` on the bare zone. An accepted goal
LANDS IN A DRAWN CELL if its cell is lethal in the mask, and FAILS TO PLAN if
its cell and every point NavFn's tolerance loop visits are lethal (Humble's
navfn_planner.cpp: x and y each from goal - tolerance, in float steps of the
resolution, while <= goal + tolerance). Only the mask is modelled: the rest of
the map is free, so these count the mask's failures alone.

    python evaluations/nogo-3199/goals.py            # writes summary.json here
"""

import json
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "service/slam/src/picar_bridge"))
from picar_bridge import keepout  # noqa: E402
from world import nogo  # noqa: E402

NAVFN_TOLERANCE_M = 0.10      # nav2.yaml GridBased.tolerance
SEED = 3199
N = 7200


def sample(rng: random.Random, d_lo: float, d_hi: float) -> dict:
    """One case: the zone (house frame), the anchor, the phase, the goal."""
    if rng.random() < 0.25:
        w, h = rng.uniform(0.20, 3.0), nogo.WALL_WIDTH_M
        if rng.random() < 0.5:
            w, h = h, w
    else:
        w, h = rng.uniform(0.20, 3.0), rng.uniform(0.20, 3.0)
    x0, y0 = rng.uniform(-10, 10), rng.uniform(-10, 10)
    zone = {"id": "z1", "x0": x0, "y0": y0, "x1": x0 + w, "y1": y0 + h}
    d = rng.uniform(d_lo, d_hi)
    # a uniform point on the curve at distance d: four edges, four quarter arcs
    per = 2 * (w + h) + 2 * math.pi * d
    s = rng.uniform(0, per)
    gx = gy = None
    for (ax, ay, bx, by, nx, ny, cx, cy, a0) in (
            (x0, y0, x0 + w, y0, 0, -1, x0 + w, y0, -math.pi / 2),
            (x0 + w, y0, x0 + w, y0 + h, 1, 0, x0 + w, y0 + h, 0.0),
            (x0 + w, y0 + h, x0, y0 + h, 0, 1, x0, y0 + h, math.pi / 2),
            (x0, y0 + h, x0, y0, -1, 0, x0, y0, math.pi)):
        L = math.hypot(bx - ax, by - ay)
        if s < L:
            t = s / L
            gx, gy = ax + t * (bx - ax) + d * nx, ay + t * (by - ay) + d * ny
            break
        s -= L
        arc = d * math.pi / 2
        if s < arc:
            ang = a0 + s / d
            gx, gy = cx + d * math.cos(ang), cy + d * math.sin(ang)
            break
        s -= arc
    if gx is None:                       # float edge: the curve's last point
        gx, gy = x0 - d, y0
    anchor = {"theta": rng.uniform(-math.pi, math.pi), "flip": rng.random() < 0.5,
              "tx": rng.uniform(-50, 50), "ty": rng.uniform(-50, 50)}
    phase = (rng.uniform(0, keepout.RES_M), rng.uniform(0, keepout.RES_M))
    return {"zone": zone, "d": d, "goal": (gx, gy), "anchor": anchor, "phase": phase}


def to_map(anchor: dict, x: float, y: float) -> tuple:
    if anchor["flip"]:
        y = -y
    c, s = math.cos(anchor["theta"]), math.sin(anchor["theta"])
    return (c * x - s * y + anchor["tx"], s * x + c * y + anchor["ty"])


def drawn_lookup(case: dict):
    """A function (x, y) map frame -> True where the bridge's mask is lethal."""
    z = case["zone"]
    corners = [(z["x0"], z["y0"]), (z["x1"], z["y0"]), (z["x1"], z["y1"]), (z["x0"], z["y1"])]
    poly = [list(to_map(case["anchor"], *p)) for p in corners]
    zones = keepout.with_margin(keepout.parse_zones([{"id": z["id"], "poly": poly}]))
    m = keepout.mask([], zones=zones, phase=case["phase"])
    ox, oy, res, w, h = m["origin_x_m"], m["origin_y_m"], m["resolution_m"], m["width"], m["height"]
    data = m["data"]

    def lethal(x: float, y: float) -> bool:
        i, j = math.floor((x - ox) / res), math.floor((y - oy) / res)
        return 0 <= i < w and 0 <= j < h and data[j * w + i] == 100
    return lethal


def navfn_plans(lethal, gx: float, gy: float, tol: float = NAVFN_TOLERANCE_M,
                res: float = keepout.RES_M) -> bool:
    """NavFn's goal: its own cell, else the nearest free point its float
    tolerance loop visits (navfn_planner.cpp, createPlan)."""
    if not lethal(gx, gy):
        return True
    y = gy - tol
    while y <= gy + tol:
        x = gx - tol
        while x <= gx + tol:
            if not lethal(x, y):
                return True
            x += res
        y += res
    return False


def judge(case: dict, rule: str, lethal=None) -> dict:
    """One goal under one rule: refused, or accepted and where it lands."""
    gx, gy = case["goal"]
    zones = [case["zone"]]
    refused = (nogo.goal_zone(zones, gx, gy) if rule == "3.199"
               else nogo.zone_at(zones, gx, gy)) is not None
    if refused:
        return {"refused": True, "in_drawn": False, "fails": False}
    lethal = lethal or drawn_lookup(case)
    mx, my = to_map(case["anchor"], gx, gy)
    in_drawn = lethal(mx, my)
    return {"refused": False, "in_drawn": in_drawn, "fails": not navfn_plans(lethal, mx, my)}


def run(n: int = N, seed: int = SEED, d_lo: float = 0.0, d_hi: float = 0.15) -> dict:
    rng = random.Random(seed)
    out = {r: {"refused": 0, "accepted": 0, "in_drawn": 0, "fails": 0, "fail_cases": []}
           for r in ("dev", "3.199")}
    same = 0
    for k in range(n):
        case = sample(rng, d_lo, d_hi)
        lethal = drawn_lookup(case)
        ans = {}
        for rule in ("dev", "3.199"):
            j = judge(case, rule, lethal)
            ans[rule] = j["refused"]
            o = out[rule]
            o["refused" if j["refused"] else "accepted"] += 1
            o["in_drawn"] += j["in_drawn"]
            o["fails"] += j["fails"]
            if j["fails"] and len(o["fail_cases"]) < 10:
                o["fail_cases"].append({"k": k, "d_m": round(case["d"], 5),
                                        "goal": [round(v, 5) for v in case["goal"]]})
        same += ans["dev"] == ans["3.199"]
    return {"n": n, "seed": seed, "d_m": [d_lo, d_hi], "same_answer": same, **out}


def main():
    near = run(N, SEED, 0.0, 0.15)
    far = run(N, SEED + 1, nogo.GOAL_CLEAR_M, 1.0)
    summary = {"goal_clear_m": nogo.GOAL_CLEAR_M, "grow_m": keepout.GROW_M,
               "navfn_tolerance_m": NAVFN_TOLERANCE_M, "near": near, "far": far}
    (Path(__file__).parent / "summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    for name, r in (("near (0, 0.15]", near), ("far (0.125, 1.0]", far)):
        print(name, {k: {kk: vv for kk, vv in v.items() if kk != "fail_cases"}
                     if isinstance(v, dict) else v for k, v in r.items()})


if __name__ == "__main__":
    main()
