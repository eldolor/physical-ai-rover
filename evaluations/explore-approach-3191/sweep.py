# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/explore-approach-3191/sweep.py -- 3.191: explore's approach
livelock. 3.70.M2's 20 unsurveyed finds (its plan, pinned in
`evaluations/find-last-seen-370/rows.jsonl` row 0: the same starts, targets
and detector seeds), run in process through M2's own `find()` (MissionRunner
-> explore -> robot/safety.py -> MockRobot on the sim clock), under a
navigator (`inflated`, M2's run 1; `fake`, its run 2), with the brain of a
checkout ROOT (this branch, or a detached `dev` worktree for the baseline).
Criteria are in `docs/plans/ros-alignment/3.191-explore-approach-livelock.md`.

    python evaluations/explore-approach-3191/sweep.py run ROOT ROWS.jsonl NAV [WORKERS [K ...]]
    python evaluations/explore-approach-3191/sweep.py score FIX.jsonl DEV.jsonl
"""

import importlib.util
import json
import math
import statistics
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
PLAN_ROWS = HERE.parents[1] / "evaluations" / "find-last-seen-370" / "rows.jsonl"
STUCK_M = 0.15            # explore's STUCK_MOVED_M
PLACE_M = 0.6             # RetryBook's radius
BAR_PER_PLACE = 3         # RETRY_LIMIT


def _plan():
    with open(PLAN_ROWS) as f:
        head = json.loads(f.readline())
    return head["plan"], head["reported"]


def _one(args):
    root, m, nav = args
    sys.path.insert(0, root)
    import brain.explore as ex
    spec = importlib.util.spec_from_file_location(
        "m2sweep", Path(root) / "evaluations" / "find-last-seen-370" / "sweep.py")
    sw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sw)
    assert Path(ex.__file__).resolve().is_relative_to(Path(root).resolve()), ex.__file__
    ended = []
    orig = ex.ExploreAgent._goal_ended

    def watched(self, state, now):
        g, pose = self._goal or {}, self._pose()
        if g.get("kind") == "approach" and state == "succeeded" and g.get("from") and pose.get("usable"):
            moved = math.hypot(pose["x_m"] - g["from"][0], pose["y_m"] - g["from"][1])
            ended.append({"target": list(g.get("target") or (g["x_m"], g["y_m"])),
                          "moved_m": moved})
        return orig(self, state, now)
    ex.ExploreAgent._goal_ended = watched
    _, reported = _plan()
    row = sw.find(m, "unsurveyed", tempfile.mkdtemp(), None, reported, nav=nav)
    still = [e["target"] for e in ended if e["moved_m"] < STUCK_M]
    places = []
    for t in still:
        for p in places:
            if math.dist(p["at"], t) <= PLACE_M:
                p["n"] += 1
                break
        else:
            places.append({"at": t, "n": 1})
    row.update(nav=nav, approach_succeeded=len(ended), approach_no_move=len(still),
               no_move_max_per_place=max((p["n"] for p in places), default=0))
    return row


def run(root, out, nav, workers=4, ks=None):
    import multiprocessing as mp
    if Path(out).exists():
        sys.exit(f"{out} exists: rows are never overwritten")
    plan, _ = _plan()
    jobs = [(str(Path(root).resolve()), m, nav) for m in plan if ks is None or m["k"] in ks]
    ctx = mp.get_context("spawn")             # each job imports ROOT's brain fresh
    with open(out, "w") as f, ctx.Pool(workers, maxtasksperchild=1) as pool:
        for r in pool.imap_unordered(_one, jobs):
            f.write(json.dumps(r) + "\n")
            f.flush()
            print(nav, r["k"], r["target"], r["outcome"], r["search_s"], r["goals"],
                  r["approach_no_move"], r["no_move_max_per_place"], flush=True)


def _rows(path):
    with open(path) as f:
        return sorted((json.loads(x) for x in f if x.strip()), key=lambda r: r["k"])


def score(fix_path, dev_path):
    fix, dev = _rows(fix_path), _rows(dev_path)
    by = {r["k"]: r for r in dev}

    def found(rows):
        return sum(r["outcome"] == "found" and not r["wrong_found"] for r in rows)
    out = {"nav": fix[0]["nav"], "missions": [len(fix), len(dev)]}
    for name, rows in (("fix", fix), ("dev", dev)):
        out[name] = {"found": found(rows), "wrong_found": sum(r["wrong_found"] for r in rows),
                     "contacts": sum(r["contacts"] for r in rows),
                     "capped": sum(r["capped"] for r in rows),
                     "no_move_max_per_place": max(r["no_move_max_per_place"] for r in rows),
                     "over_bar": [r["k"] for r in rows if r["no_move_max_per_place"] > BAR_PER_PLACE],
                     "goals_max": max(r["goals"] for r in rows),
                     "median_goals": statistics.median(r["goals"] for r in rows),
                     "median_distance_m": statistics.median(r["distance_m"] for r in rows)}
    out["differs"] = [{"k": r["k"], "target": r["target"],
                       "fix": [r["outcome"], r["search_s"], r["goals"], r["distance_m"]],
                       "dev": [by[r["k"]]["outcome"], by[r["k"]]["search_s"], by[r["k"]]["goals"],
                               by[r["k"]]["distance_m"]]}
                      for r in fix if r["k"] in by and r["outcome"] != by[r["k"]]["outcome"]]
    out["stability_met"] = len(fix) == 20 and not out["fix"]["over_bar"]
    out["progress_met"] = (len(fix) == len(dev) == 20 and out["fix"]["found"] >= out["dev"]["found"]
                           and out["fix"]["wrong_found"] == 0 and out["fix"]["contacts"] == 0)
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    a = sys.argv[1:]
    if a and a[0] == "run":
        run(a[1], a[2], a[3], int(a[4]) if len(a) > 4 else 4,
            {int(k) for k in a[5:]} or None)
    elif a and a[0] == "score":
        score(a[1], a[2])
    else:
        sys.exit(__doc__)
