# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3196/score3.py -- 3.196.B's bars (G1-G8) from 3.192's
harness outputs, into `summary.json`, which tests/test_nogo_abort_3196.py
pins.

    python evaluations/nogo-3196/score3.py DIR > DIR/summary.json

DIR holds what 3.192's `score2.py` reads (this branch's find_rule.json,
find_rooms.json, survey_rule.json, survey_rooms.json, leave.json; `dev`'s
no-zone arms dev_find_rule.json, dev_find_rooms.json, all made with
`evaluations/nogo-3192/sweep2.py`). 3.192's own records,
`evaluations/nogo-3192/`, are the baselines for G1-G3 and G7.
"""

import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.realpath(os.path.join(HERE, "..", ".."))
BASE = os.path.join(ROOT, "evaluations", "nogo-3192")

_spec = importlib.util.spec_from_file_location("score2", os.path.join(BASE, "score2.py"))
s2 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(s2)

# 3.192's records (evaluations/nogo-3192/summary.json), the bars of G2/G3.
RECORD = {"rule": {"found": 86, "median_s": 130.7}, "rooms": {"found": 81, "median_s": 111.7}}


def coverage_vs(rows, base_rows):
    """G7: per house, the zones arm's coverage outside zones against 3.192's."""
    a = {r["house"]: r for r in rows if r["arm"] == "zones"}
    b = {r["house"]: r for r in base_rows if r["arm"] == "zones"}
    out = [{"house": h, "points": round(100 * (a[h]["coverage_outside"] - b[h]["coverage_outside"]), 2)}
           for h in a if h in b and a[h].get("coverage_outside") is not None
           and b[h].get("coverage_outside") is not None]
    worst = min((x["points"] for x in out), default=None)
    return {"houses": len(out), "rows": out, "worst_points": worst,
            "met": worst is not None and worst >= -1.0}


_find_bars = s2.find_bars


def _find_bars_or_none(rows):
    """`dev`'s files hold its no-zone arm only (G6 needs no more): no find bars."""
    return _find_bars(rows) if any(r["arm"] == "zones" for r in rows) else None


def score(d):
    s2.find_bars = _find_bars_or_none
    out = {k: v for k, v in s2.score(d).items() if v is not None}
    for ch in ("rule", "rooms"):
        f = out.get(f"find_{ch}")
        if f:
            rec = RECORD[ch]
            out[f"g_find_{ch}"] = {
                "found_zones": f["found_zones"], "record_found": rec["found"],
                "median_s_zones": f["median_s_zones"], "record_median_s": rec["median_s"],
                "met": f["found_zones"] >= rec["found"]
                and f["median_s_zones"] is not None and f["median_s_zones"] <= rec["median_s"]}
        sv, base = s2.load(d, f"survey_{ch}.json"), s2.load(BASE, f"survey_{ch}.json")
        if sv and base:
            out[f"g_survey_{ch}"] = coverage_vs(sv, base)
    lv = out.get("leave")
    ent = sum(out.get(f"safety_{ch}", {}).get("entries", 0) for ch in ("rule", "rooms"))
    out["g_safety"] = {
        "in_zone": sum(out.get(f"safety_{ch}", {}).get("in_zone", 0) for ch in ("rule", "rooms")),
        "in_grown_012": sum(out.get(f"safety_{ch}", {}).get("in_grown_012", 0)
                            for ch in ("rule", "rooms")),
        "entries": ent + (lv["entries_after_leaving"] if lv else 0)}
    with open(os.path.join(BASE, "summary.json")) as fh:
        base = json.load(fh)
    out["record_loops"] = {ch: base[f"loops_{ch}"] for ch in ("rule", "rooms")}
    return out


if __name__ == "__main__":
    print(json.dumps(score(sys.argv[1]), indent=1))
