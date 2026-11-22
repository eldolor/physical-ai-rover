# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/false-sighting-3213/measure.py -- 3.213.A (records only): what a
false sighting replacing a good approach costs in time, and what the false
sightings look like.

3.210.B's broad sweep (`evaluations/reseen-3210/broad.py`): `usable()[30:210]`,
180 ProcTHOR val houses, seed 0, red backpack, no zones, through
`evaluations/nogo-3192/fix/sweep_c1.py`'s `run` unchanged. Two arms:

* `dev`: explore as it is. Every sighting explore makes is recorded (once a
  tick: `_sighting_now` is memoised per tick) with its error from the
  target's true cell, the lidar range it was placed at, the detection's
  bearing and the detector's own distance, the robot's pose, and whether the
  lidar window was an edge. Every approach actually sent is recorded as
  `broad.py` records it (`err_m`, `replaced`, `replaced_err_m`).
* `oracle`: the same, except that a sighting more than FALSE_M from the
  target never replaces a live approach goal whose own sighting lies within
  GOOD_M of it: the tick does what explore does when a sighting has not
  moved (waits on the live goal). Ground truth decides, so no real rule can
  do better: the oracle's saving is the most any rule could buy.

The sighting is still kept as seen (`_note_seen_at` runs before the
decision) in both arms; only the goal replacement is withheld.

    SWEEP_WORKERS=4 python evaluations/false-sighting-3213/measure.py OUT.json \\
        --chooser rooms|rule --arm dev|oracle|c1|c2|c1c2

Run from the checkout under test (the harness refuses another's packages).
"""

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.realpath(os.path.join(HERE, "..", ".."))
FIRST, LAST = 30, 210
GOOD_M, FALSE_M = 0.5, 1.0          # 3.210's sizing: good within 0.5 m, false beyond 1.0 m
SUPPORT_M = 0.5                     # 3.213.B C1: a place's support radius
ARMS = ("dev", "oracle", "c1", "c2", "c1c2")


def _setup(chooser, arm):
    os.environ["SWEEP2_CHOOSER"] = chooser
    sys.path.insert(0, os.path.join(ROOT, "evaluations", "nogo-3192", "fix"))
    import sweep_c1                                        # noqa: F401 -- patches sweep2/sweep
    import sweep2
    import brain.explore as ex
    from brain.arrival import ArrivalCheck
    sw = sweep2.sw
    if getattr(ex.ExploreAgent, "_measure3213", False):
        return sw
    state = {"truth": None, "sightings": None, "approaches": None, "kept": 0, "world": None}

    def err(p):
        return round(min(math.hypot(p[0] - x, p[1] - y) for x, y in state["truth"]), 2)

    def hit(p):
        """What the sighting point lies on in the house as built: the nearest
        object within one cell, else `wall` if a wall cell is within one,
        else `free`."""
        g = state["world"]
        cx, cy = int(p[0] // sw.CELL), int(p[1] // sw.CELL)
        near = [(abs(dx) + abs(dy), g.objects[(cx + dx, cy + dy)])
                for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (cx + dx, cy + dy) in g.objects]
        if near:
            return min(near)[1]
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if (0 <= cy + dy < len(g.layout) and 0 <= cx + dx < len(g.layout[0])
                        and g.layout[cy + dy][cx + dx] == "#"):
                    return "wall"
        return "free"

    orig_now = ex.ExploreAgent._sighting_now
    from brain import inventory as inv

    def _sighting_now(self, frame):
        out = orig_now(self, frame)
        if out is not None and arm in ("c1", "c2", "c1c2"):
            # 3.213.B probes: every sighting counted (one a tick: memoised),
            # and whether this tick's gate straddles a depth edge (3.204.B).
            sup = self.__dict__.setdefault("_p3213_support", [])
            sup.append(out)
            hits = self._hits(frame)
            straddles = False
            try:
                if hits:
                    straddles = inv.straddles_edge(inv.scan_returns(self.robot.get_scan()),
                                                   hits[0]["bearing_deg"], inv.EDGE_GAP_M)
            except Exception:  # noqa: BLE001 -- no scan: no edge known
                straddles = False
            self._p3213_edge = (self._ticks, out, straddles)
        if out is not None and state["sightings"] is not None and state["truth"]:
            pose = self._pose()
            hits = self._hits(frame)
            bearing = hits[0]["bearing_deg"] if hits else None
            edge = None
            try:
                scan = self.robot.get_scan()
                if bearing is not None and scan.get("usable"):
                    edge = bool(ArrivalCheck._is_edge(ArrivalCheck._window(scan, bearing)))
            except Exception:  # noqa: BLE001 -- a record only
                pass
            state["sightings"].append({
                "t": round(self.clock(), 1), "tick": self._ticks,
                "at": [round(out[0], 2), round(out[1], 2)], "err_m": err(out),
                "range_m": round(math.hypot(out[0] - pose["x_m"], out[1] - pose["y_m"]), 2),
                "bearing_deg": bearing,
                "det_dist_m": hits[0].get("distance_m") if hits else None,
                "edge": edge, "hit": hit(out),
                "from": [round(pose["x_m"], 2), round(pose["y_m"], 2)]})
        return out

    orig_send = ex.ExploreAgent._send_approach

    def _send_approach(self, tx, ty, now, scene=None):
        live = self._goal if (self._goal or {}).get("kind") == "approach" else None
        was = (live or {}).get("target")
        keep = False
        if arm == "oracle":
            keep = (state["truth"] and was is not None
                    and err(was) <= GOOD_M and err((tx, ty)) > FALSE_M)
        elif arm in ("c1", "c2", "c1c2") and was is not None:
            if arm in ("c1", "c1c2"):
                sup = self.__dict__.get("_p3213_support", [])

                def support(p):
                    return sum(1 for q in sup if math.hypot(q[0] - p[0], q[1] - p[1]) <= SUPPORT_M)
                keep = support((tx, ty)) <= support(was)
            if arm in ("c2", "c1c2") and not keep:
                e = self.__dict__.get("_p3213_edge")
                keep = bool(e and e[0] == self._ticks and e[1] == (tx, ty) and e[2])
        if keep:
            state["kept"] += 1
            return "WAIT", True, f"nav2 live: approach (3.213 {arm} kept it)"
        out = orig_send(self, tx, ty, now, scene)
        # `kind`: `goal`, an approach goal sent (what `broad.py` counted);
        # `at_target`, the sighting was within the standoff, so the live goal
        # was cancelled and explore closes in from here (`broad.py` missed it).
        sent = bool(out) and out[0] == "GOAL"
        at_target = not sent and self._goal is None and self._approach == (tx, ty)
        if state["approaches"] is not None and state["truth"] and (sent or at_target):
            state["approaches"].append({
                "t": round(now, 1), "tick": self._ticks, "at": [round(tx, 2), round(ty, 2)],
                "kind": "goal" if sent else "at_target",
                "err_m": err((tx, ty)), "replaced": live is not None,
                "replaced_err_m": err(was) if was else None})
        return out

    ex.ExploreAgent._sighting_now = _sighting_now
    ex.ExploreAgent._send_approach = _send_approach
    ex.ExploreAgent._measure3213 = True
    run_c1 = sw.run

    def run(house, seed, a, **kw):
        from sim.maps import build_world
        g = build_world(house)
        state["truth"] = [sw._centre(c) for c in sw.target_cells(g)]
        state["world"] = g
        state["sightings"], state["approaches"], state["kept"] = [], [], 0
        try:
            row = run_c1(house, seed, a, **kw)
        finally:
            s, ap, k = state["sightings"], state["approaches"], state["kept"]
            state["sightings"] = state["approaches"] = None
        row["sightings"], row["approaches"], row["oracle_kept"] = s, ap, k
        return row

    sw.run = run
    return sw


def _job(args):
    house, chooser, arm = args
    sw = _setup(chooser, arm)
    r = sw.run(house, 0, "none")
    print(f"{chooser:5} {arm:6} {house:20} {r['outcome']:13} sim {r.get('sim_s')} "
          f"kept {r['oracle_kept']} {r['wall_s']} s", file=sys.stderr, flush=True)
    return r


def main(argv):
    out = argv[0]
    chooser = argv[argv.index("--chooser") + 1] if "--chooser" in argv else "rooms"
    arm = argv[argv.index("--arm") + 1] if "--arm" in argv else "dev"
    if chooser not in ("rooms", "rule") or arm not in ARMS:
        raise SystemExit("--chooser rooms|rule --arm " + "|".join(ARMS))
    _setup(chooser, arm)
    from sim.maps.procthor import usable
    houses = [f"procthor:val:{i}" for i in usable()[FIRST:LAST]]
    workers = int(os.environ.get("SWEEP_WORKERS", "3"))
    t = time.time()
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(workers) as pool:
        rows = list(pool.map(_job, [(h, chooser, arm) for h in houses]))
    for r in rows:
        r.pop("zones", None)
    with open(out, "w") as fh:
        json.dump({"what": "3.213.A", "chooser": chooser, "arm": arm, "rows": rows,
                   "wall_s": round(time.time() - t, 1)}, fh)
    print(f"{chooser} {arm}: {len(rows)} rows in {time.time() - t:.0f} s -> {out}", file=sys.stderr)


if __name__ == "__main__":
    main(sys.argv[1:])
