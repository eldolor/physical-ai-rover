# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3197/sweep.py -- 3.197's in-process sweeps of the bridge's
zone step (`keepout`), on ground truth: robots placed around no-go zones on
many lattice phases and headings, the bridge's whole step run for each
(margins, then D4, then -- on 3.197's code -- the vent), and the drawn mask
read back.

Per position it records whether a zone was left out WHOLE because it boxed
the robot (not for depth, not fresh), whether the robot can plan out of the
mask (a 4-connected walk through undrawn cells from the robot's own cell --
NavFn frees that one -- to a cell VENT_OPEN_CELLS away), and what the vent
freed. Runs on `dev`'s keepout (no `keepout.step`) for the baseline and on
3.197's for the result.

    python evaluations/nogo-3197/sweep.py OUT.json
"""

import json
import math
import random
import sys
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "service/slam/src/picar_bridge"))
from picar_bridge import keepout  # noqa: E402

RES = keepout.RES_M
G = keepout.GROW_M
HEADINGS = [0.0, 15.0, 30.0, 45.0, 60.0, 75.0]
PHASES = [(0.0, 0.0), (0.013, 0.037), (0.049, 0.001), (0.031, 0.022)]
FAR_M = 0.60             # a vent frees nothing farther than this from the robot
OPEN_CELLS = 10           # a walk this far (Chebyshev, cells) from the robot's cell is out


def rotated(cx, cy, w, h, deg, zid):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    pts = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    return {"id": zid, "poly": [[cx + c * x - s * y, cy + s * x + c * y] for x, y in pts]}


def zones_of(*zs):
    return keepout.with_margin(keepout.parse_zones(list(zs)))


def new_state():
    return {"suspended": set(), "bare": set(), "vent": None}


def step(zs, state, pose, phase, fresh=()):
    """The bridge's _zones_step: keepout.step on 3.197's code; on dev's, the
    3.195 composition (margins_off, then suspend on the zones as drawn)."""
    if hasattr(keepout, "step"):
        return keepout.step(zs, state, pose, phase, fresh)
    off = keepout.margins_off(zs, state["bare"], state["suspended"], pose, phase, fresh=fresh)
    after = keepout.suspend(keepout.bare(zs, off), state["suspended"], pose, fresh, phase=phase)
    return {"suspended": after, "bare": off - after, "vent": None}


def drawn_mask(zs, state, phase):
    drawn = [z for z in keepout.bare(zs, state["bare"]) if z["id"] not in state["suspended"]]
    kw = {"holes": state["vent"]} if state.get("vent") else {}
    return keepout.mask([], zones=drawn, phase=phase, **kw)


def lethal(m, i, j):
    """Global lattice cell (i, j) in the mask m."""
    oi = round((m["origin_x_m"] - PH[0]) / RES)
    oj = round((m["origin_y_m"] - PH[1]) / RES)
    li, lj = i - oi, j - oj
    if not (0 <= li < m["width"] and 0 <= lj < m["height"]):
        return False
    return m["data"][lj * m["width"] + li] == 100


PH = (0.0, 0.0)


def plans_out(m, x, y, phase):
    """A 4-connected walk through undrawn cells from the robot's own cell
    (NavFn frees it) to a cell OPEN_CELLS away."""
    global PH
    PH = phase
    i0, j0 = math.floor((x - phase[0]) / RES), math.floor((y - phase[1]) / RES)
    seen, q = {(i0, j0)}, deque([(i0, j0)])
    while q:
        i, j = q.popleft()
        if max(abs(i - i0), abs(j - j0)) >= OPEN_CELLS:
            return True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            c = (i + di, j + dj)
            if c not in seen and not lethal(m, *c):
                seen.add(c)
                q.append(c)
    return False


def boxing_suspensions(zs, before, after, pose):
    """Zones left out whole this step for boxing: newly suspended, not deeper
    than SUSPEND_DEPTH_M, not fresh."""
    out = []
    for z in zs:
        if z["id"] in after and z["id"] not in before["suspended"]:
            if keepout.zone_distance(z, *pose) >= -keepout.SUSPEND_DEPTH_M:
                out.append(z["id"])
    return out


def vent_far(zs, state, pose, phase):
    """The vent's cells, and the largest distance from the robot's centre
    to a vent cell's far corner."""
    v = state.get("vent") or ()
    far = 0.0
    for i, j in v:
        for ci in (0, 1):
            for cj in (0, 1):
                far = max(far, math.hypot(phase[0] + (i + ci) * RES - pose[0],
                                          phase[1] + (j + cj) * RES - pose[1]))
    return len(v), far


def zone_points_lethal(zs, state, phase, pose, rng, n=60):
    """Points inside each zone farther than FAR_M from the robot: all
    lethal while it is drawn (the vent frees only the robot's neighbourhood)."""
    m = drawn_mask(zs, state, phase)
    global PH
    PH = phase
    bad = 0
    for z in zs:
        if z["id"] in state["suspended"]:
            continue
        p = z["poly"]
        xs, ys = [q[0] for q in p], [q[1] for q in p]
        for _ in range(n):
            x, y = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
            if keepout.zone_distance(z, x, y) >= 0 or math.hypot(x - pose[0], y - pose[1]) <= FAR_M:
                continue
            if not lethal(m, math.floor((x - phase[0]) / RES), math.floor((y - phase[1]) / RES)):
                bad += 1
    return bad


def sweep_s1(rng):
    """S1: one zone per heading and phase, 500 robots each from 0.05 m
    inside the zone to 0.04 m outside its drawn (grown) edge, around the
    whole boundary (edges and corners): 6 x 4 x 500 = 12,000."""
    rec = {"positions": 0, "boxing_suspensions": 0, "boxing_suspensions_outside": 0,
           "cannot_plan_out": 0, "vents": 0, "vent_cells_max": 0, "vent_far_m_max": 0.0, "zone_points_unlethal": 0,
           "by_heading": {}}
    for h in HEADINGS:
        hb = rec["by_heading"].setdefault(str(h), {"positions": 0, "boxing_suspensions": 0})
        for phase in PHASES:
            zs = zones_of(rotated(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0.4, 1.4),
                                  rng.uniform(0.4, 1.4), h + rng.uniform(-2, 2), "z"))
            got = 0
            while got < 500:
                bx = [q[0] for q in zs[0]["drawn"]]
                by = [q[1] for q in zs[0]["drawn"]]
                x = rng.uniform(min(bx) - 0.05, max(bx) + 0.05)
                y = rng.uniform(min(by) - 0.05, max(by) + 0.05)
                d = keepout.zone_distance(zs[0], x, y)
                if not -keepout.SUSPEND_DEPTH_M <= d <= G + 0.04:
                    continue
                got += 1
                st0 = new_state()
                st = step(zs, st0, (x, y), phase)
                rec["positions"] += 1
                hb["positions"] += 1
                b = boxing_suspensions(zs, st0, st["suspended"], (x, y))
                rec["boxing_suspensions"] += bool(b)
                rec["boxing_suspensions_outside"] += bool(b) and d > 0
                hb["boxing_suspensions"] += bool(b)
                m = drawn_mask(zs, st, phase)
                rec["cannot_plan_out"] += not plans_out(m, x, y, phase)
                if st.get("vent"):
                    rec["vents"] += 1
                    n, far = vent_far(zs, st, (x, y), phase)
                    rec["vent_cells_max"] = max(rec["vent_cells_max"], n)
                    rec["vent_far_m_max"] = round(max(rec["vent_far_m_max"], far), 4)
                    rec["zone_points_unlethal"] += zone_points_lethal(zs, st, phase, (x, y), rng)
    return rec


def sweep_s1b(rng):
    """3.195's own sampling (its Left (c)): robots 0-40 mm outside the
    drawn edge of rotated zones, away from the corners: 20 zones x 600."""
    rec = {"positions": 0, "boxing_suspensions": 0, "cannot_plan_out": 0}
    for _ in range(20):
        zs = zones_of(rotated(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0.4, 1.2),
                              rng.uniform(0.4, 1.2), rng.uniform(-180, 180), "z"))
        poly = zs[0]["poly"]
        phase = (0.0, 0.0)
        for _ in range(600):
            a, b = rng.choice(list(zip(poly, poly[1:] + poly[:1])))
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            t = rng.uniform(0.10, L - 0.10) / L
            nx, ny = (b[1] - a[1]) / L, (a[0] - b[0]) / L
            off = G + rng.uniform(1e-4, 0.04)
            x, y = a[0] + t * (b[0] - a[0]) + off * nx, a[1] + t * (b[1] - a[1]) + off * ny
            st0 = new_state()
            st = step(zs, st0, (x, y), phase)
            rec["positions"] += 1
            rec["boxing_suspensions"] += bool(boxing_suspensions(zs, st0, st["suspended"], (x, y)))
            rec["cannot_plan_out"] += not plans_out(drawn_mask(zs, st, phase), x, y, phase)
    return rec


def sweep_s5(rng):
    """S5: two zones a gap apart (0-0.12 m), rotated, 60 clusters x 150
    robots near the gap and the edges = 9,000."""
    rec = {"positions": 0, "boxing_suspensions": 0, "cannot_plan_out": 0, "vents": 0}
    for n in range(60):
        h = rng.choice(HEADINGS)
        phase = rng.choice(PHASES)
        w1 = rng.uniform(0.4, 1.0)
        gap = rng.uniform(0.0, 0.12)
        c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
        off = w1 / 2 + gap + 0.3
        zs = zones_of(rotated(0.0, 0.0, w1, 0.8, h, "a"), rotated(c * off, s * off, 0.6, 0.8, h, "b"))
        got = 0
        while got < 150:
            u = rng.uniform(w1 / 2 - 0.06, w1 / 2 + gap + 0.06)
            v = rng.uniform(-0.5, 0.5)
            x, y = c * u - s * v, s * u + c * v
            if min(keepout.zone_distance(z, x, y) for z in zs) < -keepout.SUSPEND_DEPTH_M:
                continue
            got += 1
            st0 = new_state()
            st = step(zs, st0, (x, y), phase)
            rec["positions"] += 1
            rec["boxing_suspensions"] += bool(boxing_suspensions(zs, st0, st["suspended"], (x, y)))
            rec["vents"] += bool(st.get("vent"))
            rec["cannot_plan_out"] += not plans_out(drawn_mask(zs, st, phase), x, y, phase)
    return rec


def sweep_s6(rng):
    """S6: walks with the state carried, 40 walks x 250 steps of up to 1 cm
    from a point on the drawn edge (two-zone gaps in half): plan out at
    every step."""
    rec = {"steps": 0, "cannot_plan_out": 0, "boxing_suspensions": 0, "vent_changes": 0}
    for n in range(40):
        h = rng.choice(HEADINGS + [rng.uniform(-180, 180)])
        z1 = rotated(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(0.3, 1.2),
                     rng.uniform(0.3, 1.2), h, "a")
        zl = [z1]
        if n % 2:
            c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
            cx = sum(p[0] for p in z1["poly"]) / 4
            cy = sum(p[1] for p in z1["poly"]) / 4
            w = math.hypot(z1["poly"][1][0] - z1["poly"][0][0], z1["poly"][1][1] - z1["poly"][0][1])
            o = w / 2 + rng.uniform(0.0, 0.12) + 0.3
            zl.append(rotated(cx + c * o, cy + s * o, 0.6, 2.0, h, "b"))
        zs = zones_of(*zl)
        phase = rng.choice(PHASES)
        px, py = rng.choice(zs[0]["drawn"])
        x, y = px + rng.uniform(-0.05, 0.05), py + rng.uniform(-0.05, 0.05)
        st = new_state()
        for _ in range(250):
            x, y = x + rng.uniform(-0.01, 0.01), y + rng.uniform(-0.01, 0.01)
            prev = st
            st = step(zs, st, (x, y), phase)
            rec["steps"] += 1
            rec["boxing_suspensions"] += bool(boxing_suspensions(zs, prev, st["suspended"], (x, y)))
            rec["vent_changes"] += (st.get("vent") or frozenset()) != (prev.get("vent") or frozenset())
            rec["cannot_plan_out"] += not plans_out(drawn_mask(zs, st, phase), x, y, phase)
    return rec


def sweep_s7(rng):
    """S7: 900 still robots jittering 1 mm across a cell boundary (or a
    corner) near zones, 20 ticks: a change of vent or margin after tick 4 is
    a flap."""
    flaps = 0
    for _ in range(900):
        zs = zones_of(*[rotated(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(0.2, 0.8),
                                rng.uniform(0.2, 0.8), rng.choice(HEADINGS + [rng.uniform(-180, 180)]),
                                f"z{k}") for k in range(rng.randint(1, 3))])
        phase = (rng.uniform(0, RES), rng.uniform(0, RES))
        bx = phase[0] + RES * round((rng.uniform(-0.8, 0.8) - phase[0]) / RES)
        by_ = phase[1] + RES * round((rng.uniform(-0.8, 0.8) - phase[1]) / RES)
        corner = rng.random() < 0.5
        y0 = by_ if corner else rng.uniform(-0.8, 0.8)
        st, changes, last = new_state(), 0, None
        for t in range(20):
            x = bx + (0.001 if t % 2 else -0.001)
            y = y0 + ((0.001 if t % 4 < 2 else -0.001) if corner else 0.0)
            st = step(zs, st, (x, y), phase)
            key = (frozenset(st["bare"]), frozenset(st["suspended"]), frozenset(st.get("vent") or ()))
            if t >= 4 and key != last:
                changes += 1
            last = key
        flaps += changes > 0
    return {"robots": 900, "flaps": flaps}


def main(out):
    res = {"code": "3.197" if hasattr(keepout, "step") else "dev"}
    res["S1"] = sweep_s1(random.Random(31971))
    res["S1b_3195_sampling"] = sweep_s1b(random.Random(4))
    res["S5_two_zones"] = sweep_s5(random.Random(31975))
    res["S6_walks"] = sweep_s6(random.Random(31976))
    res["S7_jitter"] = sweep_s7(random.Random(31977))
    Path(out).write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main(sys.argv[1])
