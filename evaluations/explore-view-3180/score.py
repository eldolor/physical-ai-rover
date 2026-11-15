# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/explore-view-3180/score.py -- 3.180.A's bars A2-A6 from the
sweep's paired arm files (`base-<arm>.json` from a `dev` worktree,
`a-<arm>.json` from this branch), as the section file wrote them:

  A2 missions ending without `found` where the baseline found: 0 (800)
  A3 finds, each arm and mode, >= the baseline's
  A4 median goals <= the baseline's; median sim seconds <= 1.05x, over the
     pairs both found, each arm
  A5 a mission with view_reuses 0 is identical to its baseline (outcome,
     goals, metres)
  A6 view goals reused per mission: mean <= 1.0, max <= 6

    python evaluations/explore-view-3180/score.py DIR OUT.json
"""
import json
import os
import statistics
import sys

ARMS = ("dev", "none", "right", "wrong")
SKIPS = ("no_target", "no_hint", "parse_mismatch", "skipped")


def load(d, name):
    return {(r["index"], r["mode"]): r
            for r in json.load(open(os.path.join(d, f"{name}.json")))["rows"]}


def med(xs):
    return round(statistics.median(xs), 3) if xs else None


def main(d, out):
    rep, lost, reuses = {"arms": {}}, [], []
    a2 = a3 = a4 = a5 = True
    for arm in ARMS:
        B, A = load(d, f"base-{arm}"), load(d, f"a-{arm}")
        keys = sorted(k for k in B if k in A and B[k]["outcome"] not in SKIPS
                      and A[k]["outcome"] not in SKIPS)
        lost += [[arm, *k, A[k]["outcome"]] for k in keys
                 if B[k]["outcome"] == "found" and A[k]["outcome"] != "found"]
        per = {}
        for mode in ("R", "U"):
            ks = [k for k in keys if k[1] == mode]
            fb = sum(B[k]["outcome"] == "found" for k in ks)
            fa = sum(A[k]["outcome"] == "found" for k in ks)
            per[mode] = {"pairs": len(ks), "found": [fa, fb], "met": fa >= fb}
            a3 &= fa >= fb
        both = [k for k in keys if B[k]["outcome"] == "found" and A[k]["outcome"] == "found"]
        g = [med([A[k]["goals"] for k in both]), med([B[k]["goals"] for k in both])]
        s = [med([A[k]["sim_s"] for k in both]), med([B[k]["sim_s"] for k in both])]
        m4 = g[0] <= g[1] and s[0] <= 1.05 * s[1]
        a4 &= m4
        zero = [k for k in keys if (A[k].get("view_reuses") or 0) == 0]
        diff0 = [list(k) for k in zero if [A[k][f] for f in ("outcome", "goals", "metres")]
                 != [B[k][f] for f in ("outcome", "goals", "metres")]]
        a5 &= not diff0
        changed = sum(1 for k in keys if [A[k][f] for f in ("outcome", "goals", "metres")]
                      != [B[k][f] for f in ("outcome", "goals", "metres")])
        reuses += [A[k].get("view_reuses") or 0 for k in keys]
        rep["arms"][arm] = {
            "found": per, "both_found": len(both), "goals_median": g, "sim_s_median": s,
            "A4_met": m4, "reuse_zero_missions": len(zero), "reuse_zero_differ": diff0[:10],
            "missions_changed": changed,
            "view_goals_mean": [round(statistics.mean([A[k].get("view_goals") or 0 for k in keys]), 3),
                                round(statistics.mean([B[k].get("view_goals") or 0 for k in keys]), 3)]}
    a2 = not lost
    rep["A2"] = {"lost": lost, "met": a2}
    rep["A3"] = {"met": a3}
    rep["A4"] = {"met": a4}
    rep["A5"] = {"met": a5}
    rep["A6"] = {"missions": len(reuses), "mean": round(statistics.mean(reuses), 3),
                 "max": max(reuses), "with_any": sum(1 for x in reuses if x),
                 "met": statistics.mean(reuses) <= 1.0 and max(reuses) <= 6}
    json.dump(rep, open(out, "w"), indent=1)
    print(json.dumps({k: v["met"] for k, v in rep.items() if isinstance(v, dict) and "met" in v}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
