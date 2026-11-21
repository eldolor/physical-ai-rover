# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/reseen-3210/broad.py -- 3.210.B's broad progress sweep
(amendment 1): `usable()[30:210]`, 180 ProcTHOR val houses beyond 3.192's
32, seed 0 (with no zones the seed changes nothing), red backpack, no zones,
through `evaluations/nogo-3192/fix/sweep_c1.py`'s `run` unchanged (3.161's
harness, 3.192's chooser and goal ends, C1's round count).

Read only, for the go's sizing of the false short-range sightings: every
approach explore sends is recorded with its sighting's distance from the
target's true cell (`err_m`) and whether a live approach goal was replaced by
it (`replaced`, with the replaced one's `err_m`). Nothing else is changed.

    SWEEP_WORKERS=4 python evaluations/reseen-3210/broad.py OUT.json --chooser rooms|rule

Run from the checkout under test (the harness refuses another's packages);
copy this directory in for a `dev` control.
"""

import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.realpath(os.path.join(HERE, "..", ".."))
FIRST, LAST = 30, 210


def _setup(chooser):
    os.environ["SWEEP2_CHOOSER"] = chooser
    sys.path.insert(0, os.path.join(ROOT, "evaluations", "nogo-3192", "fix"))
    import sweep_c1                                        # noqa: F401 -- patches sweep2/sweep
    import sweep2
    import brain.explore as ex
    sw = sweep2.sw
    if getattr(ex.ExploreAgent, "_broad3210", False):
        return sw
    state = {"truth": None, "log": None}
    orig = ex.ExploreAgent._send_approach

    def _send_approach(self, tx, ty, now, scene=None):
        if state["log"] is not None and state["truth"]:
            err = min(math.hypot(tx - x, ty - y) for x, y in state["truth"])
            live = self._goal if (self._goal or {}).get("kind") == "approach" else None
            state["log"].append({"t": round(now, 1), "at": [round(tx, 2), round(ty, 2)],
                                 "err_m": round(err, 2), "replaced": live is not None})
        return orig(self, tx, ty, now, scene)

    ex.ExploreAgent._send_approach = _send_approach
    ex.ExploreAgent._broad3210 = True
    run_c1 = sw.run

    def run(house, seed, arm, **kw):
        from sim.maps import build_world
        g = build_world(house)
        state["truth"] = [sw._centre(c) for c in sw.target_cells(g)]
        state["log"] = []
        try:
            row = run_c1(house, seed, arm, **kw)
        finally:
            log, state["log"] = state["log"], None
        prev = None
        for a in log:                       # the replaced approach's own sighting error
            a["replaced_err_m"] = prev["err_m"] if (a["replaced"] and prev) else None
            prev = a
        row["approaches"] = log
        return row

    sw.run = run
    return sw


def _job(args):
    house, chooser = args
    sw = _setup(chooser)
    r = sw.run(house, 0, "none")
    print(f"{chooser:5} {house:20} {r['outcome']:13} goals {r.get('goals')} "
          f"reseen {r.get('reseen')} {r['wall_s']} s", file=sys.stderr, flush=True)
    return r


def main(argv):
    out = argv[0]
    chooser = argv[argv.index("--chooser") + 1] if "--chooser" in argv else "rooms"
    if chooser not in ("rooms", "rule"):
        raise SystemExit("--chooser is rooms or rule")
    sw = _setup(chooser)
    from sim.maps.procthor import usable
    houses = [f"procthor:val:{i}" for i in usable()[FIRST:LAST]]
    workers = int(os.environ.get("SWEEP_WORKERS", "3"))
    t = time.time()
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(workers) as pool:
        rows = list(pool.map(_job, [(h, chooser) for h in houses]))
    for r in rows:
        r.pop("zones", None)
    with open(out, "w") as fh:
        json.dump({"what": "broad", "chooser": chooser, "rows": rows,
                   "wall_s": round(time.time() - t, 1)}, fh)
    print(f"broad {chooser}: {len(rows)} rows in {time.time() - t:.0f} s -> {out}",
          file=sys.stderr)
    del sw


if __name__ == "__main__":
    main(sys.argv[1:])
