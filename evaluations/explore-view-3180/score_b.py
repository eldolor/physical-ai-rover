# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/explore-view-3180/score_b.py -- 3.180.B's bars B1-B6 from the
paired sweep (`base-<arm>.json` from inside a `dev` worktree, `b-<arm>.json`
from this branch), as the section file wrote them:

  B1 wrong 247 R found
  B2 right 195 R and right 194 U still found
  B3 missions lost (baseline found, B did not): 0 of 800
  B4 finds, each arm and mode, >= the baseline's
  B5 median goals <= baseline + 1; median sim seconds <= 1.10x, over pairs
     both found, each arm
  B6 `exhausted` endings, all arms, <= the baseline's

Writes OUT_DIR/score_b.json (the scores) and OUT_DIR/rows.jsonl (one row a line): only the rows the pinned
tests read -- every mission whose (outcome, goals, metres) changed, and S7's
three -- both sides, so the committed data stays small (the orchestrator,
2026-11-15). The full arm files stay out of git.

    python evaluations/explore-view-3180/score_b.py RUNS_DIR OUT_DIR
"""
import json
import os
import statistics
import sys

ARMS = ("dev", "none", "right", "wrong")
SKIPS = ("no_target", "no_hint", "parse_mismatch", "skipped")
TRACED = {("right", 195, "R"), ("right", 194, "U"), ("wrong", 247, "R")}
KEEP = ("index", "mode", "target", "outcome", "goals", "metres", "sim_s", "hint",
        "view_goals", "view_reuses")
F = ("outcome", "goals", "metres")


def load(d, name):
    return {(r["index"], r["mode"]): r
            for r in json.load(open(os.path.join(d, f"{name}.json")))["rows"]}


def med(xs):
    return round(statistics.median(xs), 3) if xs else None


def score(arms: dict) -> dict:
    """`arms`: {arm: (base rows, B rows)}, each {(index, mode): row}."""
    rep, lost, kept = {"arms": {}}, [], []
    b4 = b5 = True
    ex_base = ex_b = 0
    for arm in ARMS:
        B, N = arms[arm]
        keys = sorted(k for k in B if k in N and B[k]["outcome"] not in SKIPS
                      and N[k]["outcome"] not in SKIPS)
        lost += [[arm, *k, N[k]["outcome"]] for k in keys
                 if B[k]["outcome"] == "found" and N[k]["outcome"] != "found"]
        per = {}
        for mode in ("R", "U"):
            ks = [k for k in keys if k[1] == mode]
            fb = sum(B[k]["outcome"] == "found" for k in ks)
            fn = sum(N[k]["outcome"] == "found" for k in ks)
            per[mode] = [fn, fb]
            b4 &= fn >= fb
        both = [k for k in keys if B[k]["outcome"] == "found" and N[k]["outcome"] == "found"]
        g = [med([N[k]["goals"] for k in both]), med([B[k]["goals"] for k in both])]
        s = [med([N[k]["sim_s"] for k in both]), med([B[k]["sim_s"] for k in both])]
        ok5 = None not in g + s and g[0] <= g[1] + 1 and s[0] <= 1.10 * s[1]
        b5 &= ok5
        eb = sum(B[k]["outcome"] == "exhausted" for k in keys)
        en = sum(N[k]["outcome"] == "exhausted" for k in keys)
        ex_base, ex_b = ex_base + eb, ex_b + en
        changed = [k for k in keys if [N[k][f] for f in F] != [B[k][f] for f in F]]
        rep["arms"][arm] = {"found_B_base": per, "both_found": len(both),
                            "goals_median": g, "sim_s_median": s, "B5_met": ok5,
                            "exhausted_B_base": [en, eb], "changed": len(changed),
                            "view_goals_mean": [round(statistics.mean([N[k].get("view_goals") or 0 for k in keys]), 3),
                                                round(statistics.mean([B[k].get("view_goals") or 0 for k in keys]), 3)]}
        for k in sorted(set(changed) | {(i, m) for a, i, m in TRACED if a == arm}):
            if k in B and k in N:
                kept.append({"arm": arm, "base": {f: B[k].get(f) for f in KEEP},
                             "b": {f: N[k].get(f) for f in KEEP}})
    out = lambda a, i, m: arms[a][1][(i, m)]["outcome"]
    rep["B1"] = {"outcome": out("wrong", 247, "R"), "met": out("wrong", 247, "R") == "found"}
    rep["B2"] = {"195R": out("right", 195, "R"), "194U": out("right", 194, "U"),
                 "met": out("right", 195, "R") == "found" and out("right", 194, "U") == "found"}
    rep["B3"] = {"lost": lost, "met": not lost}
    rep["B4"] = {"met": b4}
    rep["B5"] = {"met": b5}
    rep["B6"] = {"exhausted_B_base": [ex_b, ex_base], "met": ex_b <= ex_base}
    return rep, kept


def main(runs, out_dir):
    arms = {a: (load(runs, f"base-{a}"), load(runs, f"b-{a}")) for a in ARMS}
    rep, kept = score(arms)
    os.makedirs(out_dir, exist_ok=True)
    json.dump(rep, open(os.path.join(out_dir, "score_b.json"), "w"), indent=1)
    with open(os.path.join(out_dir, "rows.jsonl"), "w") as f:
        for row in kept:
            f.write(json.dumps(row, separators=(",", ":")) + "\n")
    print(json.dumps({k: v["met"] for k, v in rep.items() if isinstance(v, dict) and "met" in v}))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
