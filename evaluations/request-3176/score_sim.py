# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/request-3176/score_sim.py -- 3.176.B's bars S1-S7 from the
sweep's arm files (`sweep.py`), as 3.176.A wrote them:

  S1 right finds >= none's - 1, each mode
  S2 mode U, right against none: median goals <= 0.8x, median seconds
     <= 0.85x, paired mean reward 95% interval wholly above 0
  S3 mode R, right against none: median goals and seconds <= none's
  S4 wrong finds >= none's - 2, each mode, over the houses it ran
  S5 wrong / none seconds, per house where both found: median <= 1.25,
     p90 <= 2.0
  S6 0 of 200 differ (outcome, goals, metres): parsed vs none, nolabels vs
     dev, absent vs none
  S7 right and wrong end found or as none can; exceptions 0

A pair is one house in one mode; a house with no target, or that an arm left
out (no hint, a parse mismatch), is out of that arm's pairs, counted. Medians
of goals and seconds are over the pairs both arms found.

    python evaluations/request-3176/score_sim.py DIR OUT.json
"""
import json
import os
import random
import statistics
import sys

SKIPS = ("no_target", "no_hint", "parse_mismatch")
BOOT = 10_000
SEED = 3176


def load(d, arm):
    rows = json.load(open(os.path.join(d, f"{arm}.json")))["rows"]
    return {(r["index"], r["mode"]): r for r in rows}


def pairs(a, b, mode):
    """Keys both arms ran (no skip) in this mode."""
    return sorted(k for k in a if k[1] == mode and k in b
                  and a[k]["outcome"] not in SKIPS and b[k]["outcome"] not in SKIPS)


def found(r):
    return r["outcome"] == "found"


def med(xs):
    return round(statistics.median(xs), 3) if xs else None


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    i = min(len(xs) - 1, max(0, int(round(q * (len(xs) - 1)))))
    return round(xs[i], 3)


def boot_ci(diffs):
    rng = random.Random(SEED)
    n = len(diffs)
    means = sorted(sum(diffs[rng.randrange(n)] for _ in range(n)) / n for _ in range(BOOT))
    return [round(means[int(0.025 * BOOT)], 4), round(means[int(0.975 * BOOT) - 1], 4)]


def compare(a, b, mode):
    ks = pairs(a, b, mode)
    both = [k for k in ks if found(a[k]) and found(b[k])]
    return {
        "pairs": len(ks),
        "found": [sum(found(a[k]) for k in ks), sum(found(b[k]) for k in ks)],
        "both_found": len(both),
        "goals_median": [med([a[k]["goals"] for k in both]), med([b[k]["goals"] for k in both])],
        "sim_s_median": [med([a[k]["sim_s"] for k in both]), med([b[k]["sim_s"] for k in both])],
    }, ks, both


def differ(a, b):
    keys = sorted(k for k in a if k in b and a[k]["outcome"] not in SKIPS
                  and b[k]["outcome"] not in SKIPS)
    d = [k for k in keys if [a[k][f] for f in ("outcome", "goals", "metres")]
         != [b[k][f] for f in ("outcome", "goals", "metres")]]
    return {"missions": len(keys), "differ": len(d), "first": [list(k) for k in d[:5]]}


def main(d, out):
    A = {arm: load(d, arm) for arm in ("dev", "none", "right", "wrong", "absent",
                                         "nolabels", "parsed")}
    rep = {"skipped": {arm: {s: sum(r["outcome"] == s for r in rows.values()) for s in SKIPS}
                       for arm, rows in A.items()}}
    # S1, S4
    for name, arm, slack in (("S1", "right", 1), ("S4", "wrong", 2)):
        per = {}
        for mode in ("R", "U"):
            c, _ks, _b = compare(A[arm], A["none"], mode)
            per[mode] = {**c, "met": c["found"][0] >= c["found"][1] - slack}
        rep[name] = {**per, "met": all(p["met"] for p in per.values())}
    # S2: mode U
    c, ks, both = compare(A["right"], A["none"], "U")
    g, s = c["goals_median"], c["sim_s_median"]
    diffs = [A["right"][k]["reward"] - A["none"][k]["reward"] for k in ks]
    ci = boot_ci(diffs)
    rep["S2"] = {**c, "goals_ratio": round(g[0] / g[1], 3), "sim_s_ratio": round(s[0] / s[1], 3),
                 "reward_mean_diff": round(sum(diffs) / len(diffs), 4), "reward_ci95": ci,
                 "met": g[0] <= 0.8 * g[1] and s[0] <= 0.85 * s[1] and ci[0] > 0}
    # S3: mode R
    c, ks, both = compare(A["right"], A["none"], "R")
    g, s = c["goals_median"], c["sim_s_median"]
    diffs = [A["right"][k]["reward"] - A["none"][k]["reward"] for k in ks]
    rep["S3"] = {**c, "reward_mean_diff": round(sum(diffs) / len(diffs), 4),
                 "reward_ci95": boot_ci(diffs), "met": g[0] <= g[1] and s[0] <= s[1]}
    # S5
    ratios = []
    for mode in ("R", "U"):
        _c, _ks, both = compare(A["wrong"], A["none"], mode)
        ratios += [A["wrong"][k]["sim_s"] / max(A["none"][k]["sim_s"], 1e-6) for k in both]
    rep["S5"] = {"pairs_both_found": len(ratios), "median": med(ratios), "p90": pct(ratios, 0.9),
                 "worst": round(max(ratios), 3) if ratios else None}
    rep["S5"]["met"] = rep["S5"]["median"] <= 1.25 and rep["S5"]["p90"] <= 2.0
    # S6
    s6 = {"parsed_vs_none": differ(A["parsed"], A["none"]),
          "nolabels_vs_dev": differ(A["nolabels"], A["dev"]),
          "absent_vs_none": differ(A["absent"], A["none"])}
    rep["S6"] = {**s6, "met": all(v["differ"] == 0 and v["missions"] >= 190 for v in s6.values())}
    # S7
    ends = {r["outcome"] for r in A["none"].values()} | {"found"}
    odd = [[arm, *k, A[arm][k]["outcome"]] for arm in ("right", "wrong") for k in A[arm]
           if A[arm][k]["outcome"] not in SKIPS and A[arm][k]["outcome"] not in ends]
    exc = {arm: sum(r["outcome"] == "exception" for r in rows.values()) for arm, rows in A.items()}
    rep["S7"] = {"odd_outcomes": odd, "exceptions": exc,
                 "met": not odd and not any(exc.values())}
    json.dump(rep, open(out, "w"), indent=1)
    print(json.dumps({k: v.get("met") for k, v in rep.items() if isinstance(v, dict) and "met" in v}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
