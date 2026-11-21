# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/reseen-3210/score.py -- 3.210.B's bars (E1-E8) off `branch/` and
`dev/` (the control, a clean `dev` worktree, the same harness), plus the
go's read-only sizing of the false short-range sightings (broad rows).

    python evaluations/reseen-3210/score.py  -> evaluations/reseen-3210/summary.json

E3 here: every row that changed must have ended `exhausted` on `dev` having
fired a look-again round (`reseen` >= 1); every other row identical
(outcome, simulated time, goal sequence where the harness keeps it, else the
goal count). Whether a changed row's first round had left a place out is
read from its trace, recorded in the plan.
"""

import importlib.util
import json
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    "score_c1", HERE.parent / "nogo-3192" / "fix" / "score_c1.py")
sc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sc)

GOOD_M, FALSE_M = 0.5, 1.0


def _same(a, b):
    return (a["outcome"], a["sim_s"], a.get("goal_seq"), a.get("goals")) == \
        (b["outcome"], b["sim_s"], b.get("goal_seq"), b.get("goals"))


def _compare(br, dv, label):
    d = {sc._key(r): r for r in dv}
    changed, broke = [], []
    for r in br:
        o = d[sc._key(r)]
        if not _same(r, o):
            changed.append({"set": label, "mission": f"{r['house']} s{r['seed']}", "arm": r["arm"],
                            "dev": o["outcome"], "branch": r["outcome"],
                            "dev_s": o["sim_s"], "branch_s": r["sim_s"],
                            "extra_s": round(r["sim_s"] - o["sim_s"], 1),
                            "dev_reseen": o.get("reseen"), "reseen": r.get("reseen"),
                            "places": r.get("reseen_places")})
            if o["outcome"] != "exhausted" or not o.get("reseen"):
                broke.append(changed[-1])
    arms = sorted({r["arm"] for r in br})
    found = {a: {"branch": sum(r["outcome"] == "found" for r in br if r["arm"] == a),
                 "dev": sum(r["outcome"] == "found" for r in dv if r["arm"] == a)} for a in arms}
    med = {}
    for a in arms:
        for who, rows in (("branch", br), ("dev", dv)):
            t = [r["sim_s"] for r in rows if r["arm"] == a and r["outcome"] == "found"]
            med[f"{a}_{who}"] = round(statistics.median(t), 1) if t else None
    caps = [(r.get("reseen") or 0, r.get("reseen_places") or 0) for r in br]
    zone = {"goals_in_zone": 0, "goals_in_grown": 0, "entries": 0}
    for r in br:
        if r["arm"] == "zones":
            for k in zone:
                zone[k] += len(r.get(k) or [])
    return changed, broke, found, med, caps, zone


def _sizing(rows):
    n_app = sum(len(r.get("approaches") or []) for r in rows)
    false = [a for r in rows for a in r.get("approaches") or [] if a["err_m"] > FALSE_M]
    repl = [a for r in rows for a in r.get("approaches") or [] if a["replaced"]]
    good_by_false = [(r["house"], a) for r in rows for a in r.get("approaches") or []
                     if a["replaced"] and a["replaced_err_m"] is not None
                     and a["replaced_err_m"] <= GOOD_M and a["err_m"] > FALSE_M]
    hit = sorted({h for h, _ in good_by_false})
    out = {r["house"]: r["outcome"] for r in rows}
    return {"rows": len(rows), "rows_with_an_approach": sum(bool(r.get("approaches")) for r in rows),
            "approaches": n_app, "false_approaches_gt_1m": len(false),
            "replacements": len(repl), "good_replaced_by_false": len(good_by_false),
            "rows_with_good_replaced": len(hit),
            "their_outcomes": {o: sum(out[h] == o for h in hit) for o in sorted({out[h] for h in hit})}}


def main():
    b, d = HERE / "branch", HERE / "dev"
    s, changed, broke, caps, zone = {}, [], [], [], {"goals_in_zone": 0, "goals_in_grown": 0, "entries": 0}
    found, med = {}, {}
    for name in ("find_rooms", "find_rule", "broad_rooms", "broad_rule"):
        c, k, f, m, cp, z = _compare(sc._rows(b / f"{name}.json"), sc._rows(d / f"{name}.json"), name)
        changed += c
        broke += k
        caps += cp
        found.update({f"{name}_{a}": v for a, v in f.items()})
        med.update({f"{name}_{a}": v for a, v in m.items()})
        for kk in zone:
            zone[kk] += z[kk]
    e1 = [r for r in sc._rows(b / "find_rooms.json") if r["house"] == "procthor:val:44" and r["arm"] == "none"]
    s["E1"] = {"rows": [(r["seed"], r["outcome"], r["sim_s"]) for r in e1],
               "met": len(e1) == 3 and all(r["outcome"] == "found" for r in e1)}
    s["E2"] = {"found": found, "met": all(v["branch"] >= v["dev"] for v in found.values())}
    s["E3"] = {"changed_outside_dev_exhausted_reseen": broke, "met": not broke}
    s["E4"] = {"changed": changed, "all_exhausted_on_dev": all(c["dev"] == "exhausted" for c in changed),
               "median_found_s": med}
    sv = {}
    for c in ("rooms", "rule"):
        sb, sd = sc._rows(b / f"survey_{c}.json"), {sc._key(r): r for r in sc._rows(d / f"survey_{c}.json")}
        sv[c] = {"rows": len(sb), "identical": sum(sc._same(r, sd[sc._key(r)]) for r in sb)}
        for r in sb:
            if r["arm"] == "zones":
                for kk in zone:
                    zone[kk] += len(r.get(kk) or [])
    s["E5"] = {**sv, "met": all(v["rows"] == v["identical"] for v in sv.values())}
    s["E6"] = {"most_rounds": max(x[0] for x in caps), "most_places": max(x[1] for x in caps),
               "met": max(x[0] for x in caps) <= 2 and max(x[1] for x in caps) <= 6}
    s["E7"] = {**zone, "met": not any(zone.values())}
    s["E8_M3"] = sc._c8(b / "rows-m3.jsonl", d / "rows-m3.jsonl")
    s["E8_M2"] = sc._c8(b / "rows-m2.jsonl", d / "rows-m2.jsonl")
    s["sizing_dev"] = {c: _sizing(sc._rows(d / f"broad_{c}.json")) for c in ("rooms", "rule")}
    s["sizing_branch"] = {c: _sizing(sc._rows(b / f"broad_{c}.json")) for c in ("rooms", "rule")}
    (HERE / "summary.json").write_text(json.dumps(s, indent=1) + "\n")
    print(json.dumps({k: v.get("met", v) if isinstance(v, dict) else v for k, v in s.items()
                      if not k.startswith(("sizing", "E4"))}, indent=1, default=str))
    print("changed rows:", len(changed))


if __name__ == "__main__":
    sys.exit(main())
