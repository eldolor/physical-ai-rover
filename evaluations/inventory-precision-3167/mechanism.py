# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.167: why the inventory sweep's precision fell at fc5c2122 (3.74 part B).

Runs 3.60's sweep (seed 3463) from a given checkout -- that checkout's own
code and harness -- and keeps, per mission, the action sequence, each
refused move's reason, and the landmark report, so two commits compare
mission by mission.

    .venv/bin/python evaluations/inventory-precision-3167/mechanism.py run <checkout> <out.json>
    .venv/bin/python evaluations/inventory-precision-3167/mechanism.py compare <a.json> <b.json>
"""
import json
import sys
from pathlib import Path

SEED = 3463


def run(checkout, out):
    sys.path.insert(0, str(Path(checkout).resolve()))
    from tests import inventory_sweep as sw
    from sim.maps import complex_house as house
    runs = []
    for k, s in enumerate(sw.starts(seed=SEED)):
        r = sw.run_one(s, seed=SEED * 100 + k, max_steps=sw.MAX_STEPS)
        rep = r["report"]
        lms = []
        for d in rep["reported"] + rep["candidates"]:
            inst, dist = sw._nearest_of_class(d)
            lms.append({"label": d["label"], "x_m": d["x_m"], "y_m": d["y_m"],
                        "reported": d["reported"], "belief": d["belief"],
                        "right": dist <= sw.MATCH_M,
                        "nearest": None if inst is None else house.INSTANCES[inst][0]})
        runs.append({"actions": r["actions"], "landmarks": lms})
    res = {"checkout": str(checkout), "runs": runs}
    Path(out).write_text(json.dumps(res, indent=1) + "\n")


def compare(a, b):
    A, B = (json.loads(Path(p).read_text())["runs"] for p in (a, b))
    out = {"missions": len(A), "same_actions": 0, "first_divergence": [],
           "wrong_reported": {"a": [], "b": []}}
    for k, (ra, rb) in enumerate(zip(A, B)):
        if ra["actions"] == rb["actions"]:
            out["same_actions"] += 1
        else:
            i = next((i for i, (x, y) in enumerate(zip(ra["actions"], rb["actions"])) if x != y),
                     min(len(ra["actions"]), len(rb["actions"])))
            out["first_divergence"].append({"mission": k + 1, "step": i,
                                            "a": ra["actions"][i:i + 3], "b": rb["actions"][i:i + 3]})
        for side, r in (("a", ra), ("b", rb)):
            for d in r["landmarks"]:
                if d["reported"] and not d["right"]:
                    out["wrong_reported"][side].append({"mission": k + 1, **d})
    print(json.dumps(out, indent=1))
    return out


if __name__ == "__main__":
    if sys.argv[1] == "run":
        run(sys.argv[2], sys.argv[3])
    else:
        compare(sys.argv[2], sys.argv[3])
