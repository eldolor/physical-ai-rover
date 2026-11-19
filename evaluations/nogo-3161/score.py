# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3161/score.py -- 3.161.B's bars from sweep.py's output
(and the dev baseline, and live.py's), into one summary that
tests/test_no_go_sweep.py pins.

    python evaluations/nogo-3161/score.py FIND.json SURVEY.json LEAVE.json \\
        DEV_NONE.json [LIVE.json] > evaluations/nogo-3161/summary.json
"""

import json
import statistics
import sys


def load(p):
    with open(p) as fh:
        return json.load(fh)["rows"]


def score(find, survey, leave, dev, live=None):
    out = {}
    zones = [r for r in find if r["arm"] == "zones"]
    none = [r for r in find if r["arm"] == "none"]
    out["missions"] = {"zones": len(zones), "none": len(none),
                       "exceptions": sum(r["outcome"] == "exception" for r in find),
                       "capped": sum(bool(r.get("capped")) for r in find)}
    # Z1: every goal explore sent, in the zones arm (and the survey's).
    surv_z = [r for r in survey if r["arm"] == "zones"]
    goals = sum(r.get("goals", 0) for r in zones + surv_z)
    out["Z1"] = {"goals": goals,
                 "in_zone": sum(len(r.get("goals_in_zone", [])) for r in zones + surv_z),
                 "in_zone_grown_012": sum(len(r.get("goals_in_grown", [])) for r in zones + surv_z),
                 "zone_refusals_at_send": sum((r.get("nogo") or {}).get("refused", 0)
                                              for r in zones + surv_z)}
    # Z2: the true centre inside a zone after any tick.
    out["Z2"] = {"entries": sum(len(r.get("entries", [])) for r in zones + surv_z),
                 "missions_with_entry": [f"{r['house']} s{r['seed']}" for r in zones + surv_z
                                         if r.get("entries")]}
    # Z5: paired find rate and time to found (sim seconds).
    pair = {(r["house"], r["seed"]): r for r in none}
    found_z = sum(r["outcome"] == "found" for r in zones)
    found_n = sum(r["outcome"] == "found" for r in none)
    tz = [r["sim_s"] for r in zones if r["outcome"] == "found"]
    tn = [r["sim_s"] for r in none if r["outcome"] == "found"]
    lost = [f"{h} s{s}" for (h, s), n in pair.items() if n["outcome"] == "found"
            and next(z for z in zones if (z["house"], z["seed"]) == (h, s))["outcome"] != "found"]
    gained = [f"{h} s{s}" for (h, s), n in pair.items() if n["outcome"] != "found"
              and next(z for z in zones if (z["house"], z["seed"]) == (h, s))["outcome"] == "found"]
    mz, mn = (statistics.median(tz) if tz else None), (statistics.median(tn) if tn else None)
    out["Z5_find"] = {"found_zones": found_z, "found_none": found_n, "n": len(zones),
                      "lost": lost, "gained": gained,
                      "median_s_zones": mz, "median_s_none": mn,
                      "median_ratio": round(mz / mn, 4) if mz and mn else None}
    sn = {r["house"]: r for r in survey if r["arm"] == "none"}
    diffs = []
    for r in surv_z:
        n = sn[r["house"]]
        if r.get("coverage_outside") is not None and n.get("coverage_outside") is not None:
            diffs.append({"house": r["house"], "zones": r["coverage_outside"],
                          "none": n["coverage_outside"],
                          "points": round(100 * (r["coverage_outside"] - n["coverage_outside"]), 2)})
    out["Z5_survey"] = {"houses": len(diffs), "rows": diffs,
                        "worst_points_below": round(min((d["points"] for d in diffs), default=0), 2)}
    # Z3: starts inside a zone.
    rows = []
    for r in leave:
        ev = r.get("suspend_events") or []
        rest = [e for e in ev if e[0] == "restored"]
        rows.append({"house": r["house"], "left": bool(r.get("left")),
                     "restored_clear_m": rest[0][2] if rest else None,
                     "first_goal_leave": (r.get("nogo") or {}).get("leaves", 0) > 0})
    out["Z3"] = {"n": len(rows), "left": sum(x["left"] for x in rows),
                 "restored_within_030_of_clearing": sum(
                     x["restored_clear_m"] is not None and x["restored_clear_m"] - 0.30 <= 0.30 + 1e-9
                     for x in rows),
                 "rows": rows}
    # Z10: the none arm against dev's, mission by mission.
    devk = {(r["house"], r["seed"]): r for r in dev}
    same = [k for k, r in pair.items() if k in devk and r["outcome"] == devk[k]["outcome"]
            and r.get("goal_seq") == devk[k].get("goal_seq") and r.get("sim_s") == devk[k].get("sim_s")]
    out["Z10"] = {"compared": len([k for k in pair if k in devk]), "identical": len(same),
                  "differ": [f"{h} s{s}" for (h, s) in pair if (h, s) in devk and (h, s) not in same]}
    if live:
        out["live"] = {k: live.get(k) for k in ("frame_id", "z6_refused", "goals_in_zone",
                                                "peak_load", "store_after_mission")}
        out["live"]["mission"] = {k: (live.get("mission") or {}).get(k)
                                  for k in ("outcome", "seconds", "goals_sent", "entries", "nogo")}
        out["live"]["goals"] = len(live.get("goals") or [])
        out["live"]["bridge_after_draw"] = live.get("bridge_after_draw")
        out["live"]["bridge_after_mission"] = live.get("bridge_after_mission")
        out["live"]["z3"] = live.get("z3")
    return out


if __name__ == "__main__":
    a = sys.argv[1:]
    live = json.load(open(a[4])) if len(a) > 4 else None
    print(json.dumps(score(load(a[0]), load(a[1]), load(a[2]), load(a[3]), live), indent=1))
