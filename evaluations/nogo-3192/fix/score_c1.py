# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/nogo-3192/fix/score_c1.py -- 3.192.B amendment 3's bars, read off
the rows in this directory (the branch's, `sweep_c1.py` and `sweep2.py`) and
`dev/` (the control, `dev` 65f6e841 from a clean worktree, the same harness).

    python evaluations/nogo-3192/fix/score_c1.py [DIR]  -> DIR/summary.json

F1/F1': as 3.192.A amendment 1 reads them (zones found equal to no-zone;
median found time within 5%). C2: every find row (both arms, both choosers)
that does not end `exhausted` in `dev`'s control is identical (outcome,
simulated time, goal sequence). C3: survey rows identical. C4: found per arm
>= dev's. C5: goals in a zone or one grown 0.12 m, entries. C6: the most
non-succeeded goal ends within 0.5 m of one zone in a mission. C7: rounds and
places a mission, and the extra simulated time of every row C1 changed.
C8: M2's and M3's rows (`rows-m2.jsonl`, `rows-m3.jsonl`) against dev's.
"""

import json
import statistics
import sys
from pathlib import Path


def _rows(p):
    return json.loads(Path(p).read_text())["rows"]


def _key(r):
    return (r["house"], r["seed"], r["arm"])


def _f1(rows):
    found = {a: [r for r in rows if r["arm"] == a and r["outcome"] == "found"] for a in ("zones", "none")}
    mz = statistics.median(r["sim_s"] for r in found["zones"])
    mn = statistics.median(r["sim_s"] for r in found["none"])
    by = {}
    for r in rows:
        by.setdefault((r["house"], r["seed"]), {})[r["arm"]] = r["outcome"]
    lost = sorted(f"{h} s{s}" for (h, s), a in by.items() if a["none"] == "found" != a["zones"])
    gained = sorted(f"{h} s{s}" for (h, s), a in by.items() if a["zones"] == "found" != a["none"])
    ratio = mz / mn
    return {"found_zones": len(found["zones"]), "found_none": len(found["none"]),
            "median_zones_s": round(mz, 1), "median_none_s": round(mn, 1),
            "time_ratio": round(ratio, 4), "lost": lost, "gained": gained,
            "rate_met": len(found["zones"]) == len(found["none"]), "time_met": ratio <= 1.05}


def _same(a, b):
    return (a["outcome"], a["sim_s"], a.get("goal_seq")) == (b["outcome"], b["sim_s"], b.get("goal_seq"))


def _jsonl(p):
    return [json.loads(x) for x in Path(p).read_text().splitlines() if x.strip()]


def _c8(br, dv):
    def k(r):
        return (r.get("mission"), r.get("arm"), r.get("k"))
    d = {k(r): r for r in _jsonl(dv)}
    out, changed, broke = {}, [], []
    for r in _jsonl(br):
        o = d[k(r)]
        if r.get("arm") is not None:
            out.setdefault(r["arm"], [0, 0])
            out[r["arm"]][0] += r["outcome"] == "found"
            out[r["arm"]][1] += o["outcome"] == "found"
        fields = ("outcome", "search_s", "ticks", "goals", "first_goal")
        if any(r.get(f) != o.get(f) for f in fields):
            changed.append({"k": r.get("k"), "arm": r.get("arm"), "dev": o["outcome"], "branch": r["outcome"]})
            if o["outcome"] != "exhausted":
                broke.append(changed[-1])
    return {"found_branch_vs_dev": {a: {"branch": v[0], "dev": v[1]} for a, v in out.items()},
            "changed": changed, "changed_not_exhausted_on_dev": broke,
            "met": not broke and all(v[0] >= v[1] for v in out.values())}


def main(d):
    d = Path(d)
    dev = d / "dev"
    s = {}
    c2_broke, c4, c5, c6, c7, changed = [], {}, {"goals_in_zone": 0, "goals_in_grown": 0, "entries": 0}, 0, [], []
    for c in ("rule", "rooms"):
        br, dv = _rows(d / f"find_{c}.json"), {_key(r): r for r in _rows(dev / f"find_{c}.json")}
        s[f"F1_{c}"] = _f1(br)
        s[f"F1_{c}_dev"] = _f1(list(dv.values()))
        for r in br:
            o = dv[_key(r)]
            if not _same(r, o):
                changed.append({"mission": f"{r['house']} s{r['seed']}", "arm": r["arm"], "chooser": c,
                                "dev": o["outcome"], "branch": r["outcome"],
                                "extra_s": round(r["sim_s"] - o["sim_s"], 1),
                                "reseen": r.get("reseen"), "places": r.get("reseen_places")})
                if o["outcome"] != "exhausted":
                    c2_broke.append(changed[-1])
            if r["arm"] == "zones":
                c5["goals_in_zone"] += len(r["goals_in_zone"])
                c5["goals_in_grown"] += len(r["goals_in_grown"])
                c5["entries"] += len(r["entries"])
                c6 = max([c6] + list((r.get("ends_near_zone") or {}).values()))
            c7.append((r.get("reseen") or 0, r.get("reseen_places") or 0))
        for a in ("zones", "none"):
            c4[f"{c}_{a}"] = {"branch": sum(r["outcome"] == "found" for r in br if r["arm"] == a),
                              "dev": sum(r["outcome"] == "found" for r in dv.values() if r["arm"] == a)}
        sb, sd = _rows(d / f"survey_{c}.json"), {_key(r): r for r in _rows(dev / f"survey_{c}.json")}
        s[f"C3_{c}"] = {"rows": len(sb), "identical": sum(_same(r, sd[_key(r)]) for r in sb)}
        for r in sb:
            if r["arm"] == "zones":
                c5["goals_in_zone"] += len(r["goals_in_zone"])
                c5["goals_in_grown"] += len(r["goals_in_grown"])
                c5["entries"] += len(r["entries"])
    s["C2"] = {"changed": changed, "changed_not_exhausted_on_dev": c2_broke, "met": not c2_broke}
    s["C3_met"] = all(s[f"C3_{c}"]["rows"] == s[f"C3_{c}"]["identical"] for c in ("rule", "rooms"))
    s["C4"] = {"arms": c4, "met": all(v["branch"] >= v["dev"] for v in c4.values())}
    s["C5"] = {**c5, "met": not any(c5.values())}
    s["C6"] = {"most_near_one_zone": c6, "met": c6 <= 3}
    s["C7"] = {"most_rounds": max(x[0] for x in c7), "most_places": max(x[1] for x in c7),
               "met": max(x[0] for x in c7) <= 2 and max(x[1] for x in c7) <= 6}
    s["C8_M3"] = _c8(d / "rows-m3.jsonl", dev / "rows-m3.jsonl")
    s["C8_M2"] = _c8(d / "rows-m2.jsonl", dev / "rows-m2.jsonl")
    (d / "summary.json").write_text(json.dumps(s, indent=1) + "\n")
    print(json.dumps({k: (v.get("met") if isinstance(v, dict) and "met" in v else v)
                      for k, v in s.items() if not k.startswith("C2")}, indent=1))
    print("C2 met:", s["C2"]["met"], "changed rows:", len(changed))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parent)
