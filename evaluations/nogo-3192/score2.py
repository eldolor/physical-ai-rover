# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3192/score2.py -- 3.192.B's bars (F1, F1', F2, F3, F3', F4,
F5, F6, F10) from sweep2.py's outputs, into `summary.json`, which
tests/test_no_go_floor_sweep.py pins.

    python evaluations/nogo-3192/score2.py DIR > DIR/summary.json

DIR holds: find_rule.json, find_rooms.json (this branch), dev_find_rule.json,
dev_find_rooms.json (dev, both arms), survey_rule.json, survey_rooms.json,
leave.json, and optionally dev_survey_rooms.json.
"""

import json
import os
import statistics
import sys


def load(d, name):
    p = os.path.join(d, name)
    if not os.path.exists(p):
        return None
    with open(p) as fh:
        return json.load(fh)["rows"]


def find_bars(rows):
    zones = {(r["house"], r["seed"]): r for r in rows if r["arm"] == "zones"}
    none = {(r["house"], r["seed"]): r for r in rows if r["arm"] == "none"}
    fz = sum(r["outcome"] == "found" for r in zones.values())
    fn = sum(r["outcome"] == "found" for r in none.values())
    tz = [r["sim_s"] for r in zones.values() if r["outcome"] == "found"]
    tn = [r["sim_s"] for r in none.values() if r["outcome"] == "found"]
    mz = statistics.median(tz) if tz else None
    mn = statistics.median(tn) if tn else None
    lost = sorted(f"{h} s{s}" for (h, s), n in none.items()
                  if n["outcome"] == "found" and zones[(h, s)]["outcome"] != "found")
    gained = sorted(f"{h} s{s}" for (h, s), n in none.items()
                    if n["outcome"] != "found" and zones[(h, s)]["outcome"] == "found")
    return {"n": len(zones), "found_zones": fz, "found_none": fn,
            "lost": lost, "gained": gained,
            "lost_outcomes": {k: zones[tuple([k.rsplit(' s', 1)[0], int(k.rsplit(' s', 1)[1])])]
                              ["outcome"] for k in lost},
            "median_s_zones": mz, "median_s_none": mn,
            "median_ratio": round(mz / mn, 4) if mz and mn else None,
            "exceptions": sum(r["outcome"] == "exception" for r in rows),
            "capped": sum(bool(r.get("capped")) for r in rows),
            "rate_equal": fz == fn,
            "time_within_5pct": bool(mz and mn and abs(mz / mn - 1) <= 0.05)}


def safety(rows):
    z = [r for r in rows if r["arm"] == "zones"]
    return {"goals": sum(r.get("goals", 0) for r in z),
            "in_zone": sum(len(r.get("goals_in_zone", [])) for r in z),
            "in_grown_012": sum(len(r.get("goals_in_grown", [])) for r in z),
            "entries": sum(len(r.get("entries", [])) for r in z),
            "missions_with_entry": [f"{r['house']} s{r['seed']}" for r in z if r.get("entries")]}


def loops(rows):
    """F10: per mission and zone, unsucceeded goal ends within 0.5 m of it."""
    z = [r for r in rows if r["arm"] == "zones"]
    worst, total, where = 0, 0, None
    for r in z:
        for zid, n in (r.get("ends_near_zone") or {}).items():
            total += n
            if n > worst:
                worst, where = n, f"{r['house']} s{r['seed']} {zid}"
    return {"worst": worst, "worst_at": where, "total": total, "met": worst <= 3}


def identical(rows, dev):
    a = {(r["house"], r["seed"]): r for r in rows if r["arm"] == "none"}
    b = {(r["house"], r["seed"]): r for r in dev if r["arm"] == "none"}
    same = [k for k in a if k in b and a[k]["outcome"] == b[k]["outcome"]
            and a[k]["sim_s"] == b[k]["sim_s"] and a[k].get("goal_seq") == b[k].get("goal_seq")]
    return {"compared": len([k for k in a if k in b]), "identical": len(same),
            "differ": sorted(f"{h} s{s}" for (h, s) in a if (h, s) in b and (h, s) not in same)}


def survey_bars(rows):
    by = {}
    for r in rows:
        by.setdefault(r["house"], {})[r["arm"]] = r
    out = []
    for h, arms in by.items():
        z, n = arms.get("zones", {}), arms.get("none", {})
        if z.get("coverage_outside") is not None and n.get("coverage_outside") is not None:
            out.append({"house": h, "points": round(100 * (z["coverage_outside"]
                                                            - n["coverage_outside"]), 2)})
    worst = min((x["points"] for x in out), default=None)
    return {"houses": len(out), "rows": out, "worst_points_below": worst,
            "met": worst is not None and worst >= -1.0}


def leave_bars(rows):
    res = []
    for r in rows:
        rest = [e for e in r.get("suspend_events") or [] if e[0] == "restored"]
        res.append({"house": r["house"], "left": bool(r.get("left")),
                    "restored_clear_m": rest[0][2] if rest else None})
    ok = sum(x["left"] and x["restored_clear_m"] is not None
             and x["restored_clear_m"] <= 0.60 + 1e-9 for x in res)
    return {"n": len(res), "left": sum(x["left"] for x in res),
            "restored_within_030_of_clearing": ok, "entries_after_leaving":
            sum(len(r.get("entries", [])) for r in rows), "rows": res}


def score(d):
    out = {}
    for ch in ("rule", "rooms"):
        br, dv = load(d, f"find_{ch}.json"), load(d, f"dev_find_{ch}.json")
        sv = load(d, f"survey_{ch}.json")
        if br:
            out[f"find_{ch}"] = find_bars(br)
            out[f"safety_{ch}"] = safety(br + (sv or []))
            out[f"loops_{ch}"] = loops(br + (sv or []))
        if dv:
            out[f"dev_find_{ch}"] = find_bars(dv)
            out[f"dev_loops_{ch}"] = loops(dv)
        if br and dv:
            out[f"identical_{ch}"] = identical(br, dv)
        if sv:
            out[f"survey_{ch}"] = survey_bars(sv)
    dsv = load(d, "dev_survey_rooms.json")
    if dsv:
        out["dev_survey_rooms"] = survey_bars(dsv)
    lv = load(d, "leave.json")
    if lv:
        out["leave"] = leave_bars(lv)
    return out


if __name__ == "__main__":
    print(json.dumps(score(sys.argv[1]), indent=1))
