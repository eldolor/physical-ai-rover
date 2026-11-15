# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-synonyms-3185/score.py -- 3.185's bars S2-S4 from the sweep's
arms: `dev_<arm>.json` run inside a `dev` worktree (dev, names, names_syn),
`branch_<arm>.json` on this branch (dev, names, names_syn, names_near).

    python evaluations/room-synonyms-3185/score.py   -> score.json

A pair differs when any of outcome, goals or metres differs for the same house,
split and mode.
"""
import json
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
KEYS = ("outcome", "goals", "metres")
ARMS = [("dev", a) for a in ("dev", "names", "names_syn")] + \
       [("branch", a) for a in ("dev", "names", "names_syn", "names_near")]


def load(tree, arm):
    with open(os.path.join(HERE, f"{tree}_{arm}.json")) as f:
        return json.load(f)


def differ(a, b):
    A = {(r["split"], r["index"], r["mode"]): r for r in a["rows"]}
    B = {(r["split"], r["index"], r["mode"]): r for r in b["rows"]}
    assert set(A) == set(B), "the arms ran different missions"
    return sorted(k for k in A if any(A[k].get(x) != B[k].get(x) for x in KEYS))


def summary(rep):
    out = {}
    for m in ("R", "U"):
        rs = [r for r in rep["rows"] if r["mode"] == m and r["outcome"] != "no_target"]
        out[m] = {"missions": len(rs), "found": sum(r["outcome"] == "found" for r in rs),
                  "goals_mean": round(statistics.mean(r["goals"] or 0 for r in rs), 2),
                  "metres_mean": round(statistics.mean(r["metres"] or 0 for r in rs), 2)}
    return out


def score(arms: dict) -> dict:
    s2_names = differ(arms["dev_names"], arms["branch_names"])
    s2_none = differ(arms["dev_dev"], arms["branch_dev"])
    s3 = differ(arms["branch_names_syn"], arms["branch_names"])
    s4 = differ(arms["branch_names_near"], arms["branch_dev"])
    return {
        "brain_paths": {k: v.get("brain_path") for k, v in arms.items()},
        "S2": {"names_differ": len(s2_names), "no_names_differ": len(s2_none),
               "met": not s2_names and not s2_none},
        "S3": {"differ": len(s3), "met": not s3},
        "S4": {"differ": len(s4), "met": not s4},
        "reported": {
            "dev_syn_vs_dev_no_names": len(differ(arms["dev_names_syn"], arms["dev_dev"])),
            "dev_syn_vs_dev_names": len(differ(arms["dev_names_syn"], arms["dev_names"]))},
        "summary": {k: summary(v) for k, v in arms.items()},
    }


def main():
    arms = {f"{t}_{a}": load(t, a) for t, a in ARMS}
    report = score(arms)
    with open(os.path.join(HERE, "score.json"), "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps({k: v for k, v in report.items() if k != "summary"}, indent=1))


if __name__ == "__main__":
    main()
