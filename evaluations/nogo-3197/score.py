# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3197/score.py -- 3.197's live bars (L1-L6) from live.py's
records. Each run is read by 3.195's `score.run` (the same fields); the
summary pools the counted prefixes (`a` and `r`: L1-L5) and reports any
other prefix (`b`, dev's bridge) beside them.

    python evaluations/nogo-3197/score.py OUT.json a r [b]
"""

import glob
import importlib.util
import json
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "score3195", os.path.join(HERE, "..", "nogo-3195", "score.py"))
S5 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(S5)

COUNTED = ("a", "r")


def runs_of(prefix):
    paths = [p for p in glob.glob(os.path.join(HERE, f"{prefix}-*.json"))
             if re.fullmatch(rf"{re.escape(prefix)}-\d+\.json", os.path.basename(p))]
    out = []
    for p in sorted(paths, key=lambda p: int(p.rsplit("-", 1)[1].split(".")[0])):
        r = S5.run(p)
        d = json.load(open(p))
        log = d.get("bridge_zone_log_mission")
        r["start_turn_deg"] = d.get("start_turn_deg")
        # L5/L6: the mission window's lines (None when the log was not read)
        r["mission_whole_suspensions"] = None if log is None else sum("zone_suspended" in ln for ln in log)
        r["mission_vents"] = None if log is None else sum("zone_vent " in ln for ln in log)
        r["mission_margin_off"] = None if log is None else sum("zone_margin_off" in ln for ln in log)
        r["nogo_after_mission"] = d.get("bridge_after_mission")
        out.append(r)
    return out


def summary(runs):
    found = [r for r in runs if r["outcome"] == "found"]
    return {
        "runs": len(runs),
        "invalid_runs": [r["run"] for r in runs if not r["valid"] or (r["peak_load"] or 0) > 4.0],
        "L1_entries": sum(r["entries"] for r in runs),
        "L2_closest_true_m": [r["closest_true_m"] for r in runs],
        "L2_min_m": min((r["closest_true_m"] for r in runs if r["closest_true_m"] is not None), default=None),
        "L3_found": len(found),
        "L3_median_s": statistics.median(r["seconds"] for r in found) if found else None,
        "L3_seconds": [r["seconds"] for r in runs],
        "L3_goals_in_zone": sum(r["goals_in_zone"] or 0 for r in runs),
        "L3_docked": sum(r["docked"] for r in runs),
        "L4_both_events": sum(bool(r["z3_seen_suspended"] and r["z3_restored_after_s"] is not None)
                              for r in runs),
        "L4_restore_clear_m": [r["z3_restore_clear_m"] for r in runs],
        "L4_left_after_s": [r["z3_left_after_s"] for r in runs],
        "L4_final": [r["z3_final"] for r in runs],
        "L5_whole_suspensions": [r["mission_whole_suspensions"] for r in runs],
        "L6_vents": [r["mission_vents"] for r in runs],
        "L6_margin_off": [r["mission_margin_off"] for r in runs],
        "peak_load": max((r["peak_load"] or 0) for r in runs) if runs else None,
    }


def main(out, prefixes):
    by = {p: runs_of(p) for p in prefixes}
    counted = [r for p in prefixes if p in COUNTED for r in by[p]]
    res = {"counted": summary(counted), "by_prefix": {p: summary(rs) for p, rs in by.items()},
           "runs": {p: rs for p, rs in by.items()}}
    with open(out, "w") as fh:
        json.dump(res, fh, indent=1)
    print(json.dumps({"counted": res["counted"], "by_prefix": res["by_prefix"]}, indent=1))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
