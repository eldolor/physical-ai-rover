# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3195/score.py -- 3.195's bars from live.py's records.

    python evaluations/nogo-3195/score.py PREFIX [OUT.json]

M1 (the cause) per run: the closest true approach and the SLAM distance at
that sample, SLAM's own closest approach, and the offset's component toward
the zone. C1-C5 over the PREFIX-*.json runs.
"""

import glob
import json
import math
import os
import re
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _toward(z, s):
    """The offset SLAM - truth along the truth's direction to the zone's
    nearest edge point: positive = SLAM nearer the zone than the truth."""
    tx, ty = s["truth"]
    nx, ny = min(max(tx, z["x0"]), z["x1"]), min(max(ty, z["y0"]), z["y1"])
    dx, dy = nx - tx, ny - ty
    L = math.hypot(dx, dy)
    if L == 0:
        return None
    return round((s["offset_m"][0] * dx + s["offset_m"][1] * dy) / L, 4)


def run(path):
    d = json.load(open(path))
    m, z3 = d.get("mission") or {}, d.get("z3") or {}
    zones = {z["id"]: z for z in d.get("zones") or []}
    S = d.get("samples") or []
    c = m.get("closest")
    sm = min(S, key=lambda s: s["slam_min_d_m"]) if S else None
    near = [s for s in S if s["truth_d_m"] < 0.05]
    tow = [t for t in (_toward(zones[s["zone"]], s) for s in near) if t is not None]
    docked = any(ln.startswith("return docked:") for ln in m.get("log_tail") or [])   # review: not "not docked"
    return {
        "run": os.path.basename(path), "valid": m.get("closest_outside_m") is not None,
        "outcome": m.get("outcome"), "seconds": m.get("seconds"),
        "goals_sent": m.get("goals_sent"), "goals_in_zone": d.get("goals_in_zone"),
        "docked": docked, "peak_load": d.get("peak_load"),
        "samples": m.get("truth_samples"), "entries": len(m.get("entries") or []),
        "closest_true_m": m.get("closest_outside_m"),
        "slam_d_at_closest_m": c and c["slam_d_m"], "closest_at": c and c["truth"],
        "closest_t_s": c and c["t"],
        "slam_closest_m": sm and sm["slam_min_d_m"], "slam_closest_at": sm and sm["slam"],
        "offset_toward_zone_median_m": statistics.median(tow) if tow else None,
        "offset_median_m": m.get("offset_m_median"), "offset_max_m": m.get("offset_m_max"),
        "z3_seen_suspended": z3.get("seen_suspended"), "z3_restored_after_s": z3.get("restored_after_s"),
        "z3_restore_clear_m": z3.get("truth_clear_at_restore_m"), "z3_left_after_s": z3.get("left_after_s"),
        "z3_final": z3.get("final_outcome"), "z3_ended": z3.get("ended"),
        "z3_bridge_log": len(z3.get("bridge_log") or []),
        # C10 (amendment 1): D4 suspensions during the mission, the trip home
        # included (the bridge's counter), and amendment 2's margin events
        "mission_suspensions": (d["bridge_after_mission"]["suspended_count"]
                                - d["bridge_after_draw"]["suspended_count"]
                                if (d.get("bridge_after_mission") or {}).get("suspended_count") is not None
                                and (d.get("bridge_after_draw") or {}).get("suspended_count") is not None
                                else None),                       # review: missing is not 0
        "mission_zone_log": d.get("bridge_zone_log_mission"),
    }


def main(prefix, out=None):
    paths = [p for p in glob.glob(os.path.join(HERE, f"{prefix}-*.json"))
             if re.fullmatch(rf"{re.escape(prefix)}-\d+\.json", os.path.basename(p))]   # review
    runs = [run(p) for p in sorted(paths, key=lambda p: int(p.rsplit("-", 1)[1].split(".")[0]))]
    found = [r for r in runs if r["outcome"] == "found"]
    summary = {
        "runs": len(runs),
        "invalid_runs": [r["run"] for r in runs if not r["valid"]],     # review: never counted as 0 entries
        "C1_entries": sum(r["entries"] for r in runs),
        "C2_closest_true_m": [r["closest_true_m"] for r in runs],
        "C2_min_m": min((r["closest_true_m"] for r in runs if r["closest_true_m"] is not None), default=None),
        "C3_found": len(found),
        "C3_median_s": statistics.median(r["seconds"] for r in found) if found else None,
        "C3_goals_in_zone": sum(r["goals_in_zone"] or 0 for r in runs),
        "C3_docked": sum(r["docked"] for r in runs),
        "C4_both_events": sum(bool(r["z3_seen_suspended"] and r["z3_restored_after_s"] is not None)
                              for r in runs),
        "C4_restore_clear_m": [r["z3_restore_clear_m"] for r in runs],
        "C5_left_after_s": [r["z3_left_after_s"] for r in runs],
        "C5_final": [r["z3_final"] for r in runs],
        "slam_closest_m": [r["slam_closest_m"] for r in runs],
        "C10_mission_suspensions": [r["mission_suspensions"] for r in runs],
        "C10_mission_zone_log": [r["mission_zone_log"] for r in runs],
        "peak_load": max((r["peak_load"] or 0) for r in runs) if runs else None,
    }
    res = {"summary": summary, "runs": runs}
    if out:
        with open(out, "w") as fh:
            json.dump(res, fh, indent=1)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
