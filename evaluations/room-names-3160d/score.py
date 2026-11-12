# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/room-names-3160d/score.py -- 3.160.D's bars D1-D5 from the sweep's
arms (dev.json from `dev` before the change; names, noid, empty after it).

    python evaluations/room-names-3160d/score.py [DIR]
"""
import json
import statistics
import sys
from pathlib import Path

import numpy as np

DRAWS, SEED = 10_000, 3160


def load(d, arm):
    rows = json.loads((d / f"{arm}.json").read_text())["rows"]
    return {(r["split"], r["index"], r["mode"]): r for r in rows}


def main(d: Path) -> dict:
    dev, names = load(d, "dev"), load(d, "names")
    out = {}
    # D1: mean reward, names minus no names, mode R, bootstrap over houses.
    keys = sorted(k for k in dev if k[2] == "R" and dev[k]["outcome"] != "no_target")
    diff = np.array([names[k]["reward"] - dev[k]["reward"] for k in keys])
    rng = np.random.default_rng(SEED)
    boots = diff[rng.integers(0, len(diff), (DRAWS, len(diff)))].mean(axis=1)
    lo, hi = np.percentile(boots, [2.5, 97.5])
    out["D1"] = {"missions": len(keys), "mean_diff": round(float(diff.mean()), 4),
                 "ci95": [round(float(lo), 4), round(float(hi), 4)], "met": bool(lo > 0)}
    # D2: finds, each mode.
    d2 = {}
    for m in ("R", "U"):
        ks = [k for k in dev if k[2] == m and dev[k]["outcome"] != "no_target"]
        a = sum(dev[k]["outcome"] == "found" for k in ks)
        b = sum(names[k]["outcome"] == "found" for k in ks)
        d2[m] = {"dev": a, "names": b, "met": b >= a - 1}
    out["D2"] = {**d2, "met": all(v["met"] for v in d2.values())}
    # D3: mode U, median metres of found missions.
    mu = [k for k in dev if k[2] == "U"]
    a = statistics.median(dev[k]["metres"] for k in mu if dev[k]["outcome"] == "found")
    b = statistics.median(names[k]["metres"] for k in mu if names[k]["outcome"] == "found")
    out["D3"] = {"dev_m": a, "names_m": b, "ratio": round(b / a, 3), "met": b <= 1.10 * a}
    # D4: off is off -- no frame_id, and a frame_id with no names, equal dev.
    d4 = {}
    for arm in ("noid", "empty"):
        if not (d / f"{arm}.json").exists():
            continue
        x = load(d, arm)
        diffs = [k for k in dev if [dev[k][f] for f in ("outcome", "goals", "metres")]
                 != [x[k][f] for f in ("outcome", "goals", "metres")]]
        d4[arm] = {"missions": len(dev), "differ": len(diffs), "first": [list(k) for k in diffs[:5]]}
    out["D4"] = {**d4, "met": bool(d4) and all(v["differ"] == 0 for v in d4.values())}
    # D5: asks, and that the names were read.
    out["D5"] = {"asks": sum(r.get("asks") or 0 for r in names.values()),
                 "missions_reading_names": sum(bool(r.get("person_rooms")) for r in names.values()),
                 "exceptions": sum(r["outcome"] == "exception" for r in names.values())}
    out["D5"]["met"] = out["D5"]["asks"] == 0
    # Where it moved: per-mode means for the record.
    for m in ("R", "U"):
        ks = [k for k in dev if k[2] == m and dev[k]["outcome"] != "no_target"]
        out[f"mode_{m}"] = {arm: {"reward": round(statistics.mean(src[k]["reward"] for k in ks), 4),
                                  "goals": round(statistics.mean(src[k]["goals"] for k in ks), 2),
                                  "metres": round(statistics.mean(src[k]["metres"] for k in ks), 2),
                                  "changed": sum([src[k][f] for f in ("outcome", "goals", "metres")]
                                                 != [dev[k][f] for f in ("outcome", "goals", "metres")]
                                                 for k in ks) if arm == "names" else None}
                            for arm, src in (("dev", dev), ("names", names))}
    return out


if __name__ == "__main__":
    print(json.dumps(main(Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent)), indent=1))
