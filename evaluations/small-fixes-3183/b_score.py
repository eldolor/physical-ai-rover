# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/small-fixes-3183/b_score.py -- 3.183.B's bars B2-B3 from the six
sweep arms (`sweep_b.py`): `b_dev_<arm>.json` run inside a `dev` worktree,
`b_branch_<arm>.json` on this branch, arms `dev` (no names), `names`,
`names_title`.

    python evaluations/small-fixes-3183/b_score.py   -> b_score.json

A pair "differs" when any of outcome, goals or metres differs for the same
house, split and mode.
"""
import json
import os
import statistics

HERE = os.path.dirname(os.path.abspath(__file__))
KEYS = ("outcome", "goals", "metres")


def load(name):
    with open(os.path.join(HERE, name)) as f:
        return json.load(f)


def key(r):
    return (r["split"], r["index"], r["mode"])


def differ(a, b):
    """Rows of a and b that disagree, by (split, index, mode); both arms must
    hold the same missions."""
    A, B = {key(r): r for r in a["rows"]}, {key(r): r for r in b["rows"]}
    assert set(A) == set(B), "the arms ran different missions"
    return sorted(k for k in A if any(A[k].get(x) != B[k].get(x) for x in KEYS))


def summary(rep):
    out = {}
    for m in ("R", "U"):
        rs = [r for r in rep["rows"] if r["mode"] == m and r["outcome"] != "no_target"]
        found = [r for r in rs if r["outcome"] == "found"]
        out[m] = {"missions": len(rs), "found": len(found),
                  "goals_mean": round(statistics.mean(r["goals"] or 0 for r in rs), 2),
                  "metres_mean": round(statistics.mean(r["metres"] or 0 for r in rs), 2)}
    return out


def main():
    arms = {f"{t}_{a}": load(f"b_{t}_{a}.json")
            for t in ("dev", "branch") for a in ("dev", "names", "names_title")}
    paths = {k: v.get("brain_path") for k, v in arms.items()}
    b2_names = differ(arms["dev_names"], arms["branch_names"])
    b2_none = differ(arms["dev_dev"], arms["branch_dev"])
    b3 = differ(arms["branch_names_title"], arms["branch_names"])
    fault = differ(arms["dev_names_title"], arms["dev_names"])
    fault_vs_none = differ(arms["dev_names_title"], arms["dev_dev"])
    report = {
        "brain_paths": paths,
        "B2": {"names_differ": len(b2_names), "no_names_differ": len(b2_none),
               "missions": len(arms["dev_names"]["rows"]),
               "met": not b2_names and not b2_none, "rows": b2_names + b2_none},
        "B3": {"differ": len(b3), "missions": len(arms["branch_names"]["rows"]),
               "met": not b3, "rows": b3},
        "reported": {"dev_names_title_vs_dev_names_differ": len(fault),
                     "dev_names_title_vs_dev_no_names_differ": len(fault_vs_none)},
        "summary": {k: summary(v) for k, v in arms.items()},
    }
    with open(os.path.join(HERE, "b_score.json"), "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps({k: v for k, v in report.items() if k != "summary"}, indent=1, default=str))


if __name__ == "__main__":
    main()
