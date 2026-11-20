# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/dock-3156e/score.py -- 3.156.E's live bars scored from
`live.py`'s records (`live.jsonl`, the main run; `live-g4-300s.jsonl`, the
supplementary G4 run). Pinned by `tests/test_dock_live_3156e.py`.

    python evaluations/dock-3156e/score.py [live.jsonl [live-g4-300s.jsonl]]
"""

import json
import math
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EVERY_M, EVERY_DEG = 0.15, 15.0     # C2: every docked run
MEDIAN_M, MEDIAN_DEG = 0.10, 10.0   # C2: the median
STOP_MOVED_CM = 1.0                 # G5's "no motion": under a centimetre in the 10 s after


def rows(path):
    p = Path(path)
    return [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []


def _lapsed(r):
    ret = r.get("ret") or {}
    return "autonomy_lapsed" in f"{ret.get('reason') or ''} {r.get('end_reason') or ''}"


def _errors(found):
    return {"n": len(found),
            "docked": sum((r.get("ret") or {}).get("state") == "docked" for r in found),
            "in_every_bar": sum(bool(r.get("docked_in_bars")) for r in found),
            "median_m": round(statistics.median(r["err_m"] for r in found), 3) if found else None,
            "median_deg": round(statistics.median(r["err_deg"] for r in found), 1) if found else None,
            "worst_m": max((r["err_m"] for r in found), default=None),
            "worst_deg": max((r["err_deg"] for r in found), default=None),
            "contacts": sum(r.get("contacts") or 0 for r in found),
            "least_gap_cm": min((r["min_gap_cm"] for r in found if r.get("min_gap_cm") is not None),
                                default=None),
            "load_max": max((r.get("load_max") or 0 for r in found), default=None)}


def score(main_rows, supp_rows=()):
    finds = [r for r in main_rows if r["kind"] == "find"]
    drills = [r for r in main_rows if r["kind"] == "drill"]
    supp = [r for r in supp_rows if r["kind"] == "find"]

    g1 = [r for r in finds if r["house"] == "scaled_house"]
    docks = [r.get("dock") or {} for r in g1]
    spread_m = max((math.hypot(a["x_m"] - b["x_m"], a["y_m"] - b["y_m"])
                    for a in docks for b in docks), default=None)
    spread_deg = max((abs((a["theta"] - b["theta"] + 180) % 360 - 180)
                      for a in docks for b in docks), default=None)
    G1 = {"missions": len(g1),
          "saved_map": sum(d.get("source") == "saved_map" for d in docks),
          "off_dock_m": [r["start_off_dock_m"] for r in g1],
          "dock_spread_m": spread_m, "dock_spread_deg": spread_deg,
          **_errors(g1)}
    G1["met"] = (G1["missions"] == 5 and G1["saved_map"] == 5 and spread_m <= 0.05
                 and spread_deg <= 2.0 and G1["in_every_bar"] == 5
                 and all(1.0 <= m <= 4.0 for m in G1["off_dock_m"]))

    G3 = {"main": _errors(finds), "supplementary": _errors(supp)}
    both = finds + supp
    G3["met"] = (G3["main"]["n"] == 10 and G3["main"]["docked"] >= 9
                 and all(r["err_m"] <= EVERY_M and r["err_deg"] <= EVERY_DEG
                         for r in both if (r.get("ret") or {}).get("state") == "docked")
                 and statistics.median(r["err_m"] for r in both) <= MEDIAN_M
                 and statistics.median(r["err_deg"] for r in both) <= MEDIAN_DEG
                 and sum(r.get("contacts") or 0 for r in both) == 0)

    before = [r.get("goal_posts_before_end") for r in both]
    G4 = {"missions": len(both), "goal_posts_before_end": sum(b or 0 for b in before),
          "unmeasured": sum(b is None for b in before),
          "goal_posts_after_end": [r.get("goal_posts_after_end") for r in both],
          "end_decided_s": [r.get("end_decided_s") for r in both]}
    G4["met"] = G4["missions"] >= 10 and G4["goal_posts_before_end"] == 0 and G4["unmeasured"] == 0

    G5 = {"drills": len(drills),
          "cancelled": sum((r.get("ret") or {}).get("state") == "cancelled" for r in drills),
          "stopped_while": [r.get("drill_phase") for r in drills],
          "moved_after_stop_cm": [r.get("moved_after_stop_cm") for r in drills],
          "autonomy_lapsed_in_g3": sum(_lapsed(r) for r in both)}
    G5["met"] = (G5["drills"] == 5 and G5["cancelled"] == 5
                 and all(m is not None and m < STOP_MOVED_CM for m in G5["moved_after_stop_cm"])
                 and G5["autonomy_lapsed_in_g3"] == 0)
    return {"G1": G1, "G3": G3, "G4_live": G4, "G5": G5}


if __name__ == "__main__":
    a = sys.argv[1:]
    main_path = a[0] if a else HERE / "live.jsonl"
    supp_path = a[1] if len(a) > 1 else HERE / "live-g4-300s.jsonl"
    print(json.dumps(score(rows(main_path), rows(supp_path)), indent=1))
