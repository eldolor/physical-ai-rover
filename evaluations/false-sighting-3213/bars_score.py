# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/false-sighting-3213/bars_score.py -- 3.213.B's bars, as the plan
file writes them, off `B-bars/branch/` and `B-bars/dev/` (a clean `dev`
worktree, the same harness):

* time: broad both-found median and p90 `sim_s` <= `dev`'s under each
  chooser; the median saving over affected rows (a good approach replaced by
  a false sighting on `dev`) >= 1%;
* found >= `dev` in every arm: 3.192's four find arms and the broad sweep
  under both choosers; the val:44 s0-s2 rooms/none pin all `found`;
* wrong found 0: no `found` row (find or broad) ends more than 1.0 m from a
  target cell (`final_d_m`);
* surveys: every row identical (outcome, simulated time, goal sequence);
* 3.70.M2 and M3: found >= `dev` per arm.

The frozen references (`git diff origin/dev -- tests/*_reference.py`) are
checked by hand and recorded in the plan.

    python evaluations/false-sighting-3213/bars_score.py  -> B-bars/summary.json
"""

import importlib.util
import json
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location(
    "score_c1", HERE.parent / "nogo-3192" / "fix" / "score_c1.py")
sc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sc)
_spec2 = importlib.util.spec_from_file_location("score3213", HERE / "score.py")
s13 = importlib.util.module_from_spec(_spec2)
_spec2.loader.exec_module(s13)
WRONG_M = 1.0


def main():
    b, d = HERE / "B-bars" / "branch", HERE / "B-bars" / "dev"
    out = {}
    found, wrong = {}, []
    for c in ("rooms", "rule"):
        br, dv = sc._rows(b / f"find_{c}.json"), sc._rows(d / f"find_{c}.json")
        for a in ("none", "zones"):
            found[f"find_{c}_{a}"] = [sum(r["outcome"] == "found" for r in dv if r["arm"] == a),
                                      sum(r["outcome"] == "found" for r in br if r["arm"] == a)]
        wrong += [f"find {c} {r['house']} s{r['seed']} {r['arm']} {r.get('final_d_m')}" for r in br
                  if r["outcome"] == "found" and (r.get("final_d_m") is None or r["final_d_m"] > WRONG_M)]
        if c == "rooms":
            pin = [r for r in br if r["house"] == "procthor:val:44" and r["arm"] == "none"]
            out["pin_val44"] = {"rows": [(r["seed"], r["outcome"], r["sim_s"]) for r in pin],
                                "met": len(pin) == 3 and all(r["outcome"] == "found" for r in pin)}
    time = {}
    for c in ("rooms", "rule"):
        bb = {r["house"]: r for r in sc._rows(b / f"broad_{c}.json")}
        dd = {r["house"]: r for r in sc._rows(d / f"broad_{c}.json")}
        found[f"broad_{c}"] = [sum(r["outcome"] == "found" for r in dd.values()),
                               sum(r["outcome"] == "found" for r in bb.values())]
        wrong += [f"broad {c} {h} {r.get('final_d_m')}" for h, r in bb.items()
                  if r["outcome"] == "found" and (r.get("final_d_m") is None or r["final_d_m"] > WRONG_M)]
        both = [h for h in dd if dd[h]["outcome"] == "found" and bb[h]["outcome"] == "found"]
        aff = [h for h in dd if any(s13.gbf(a) for a in dd[h]["approaches"])]
        sav = [(dd[h]["sim_s"] - bb[h]["sim_s"]) / dd[h]["sim_s"] for h in aff if h in both]
        med = [statistics.median(dd[h]["sim_s"] for h in both), statistics.median(bb[h]["sim_s"] for h in both)]
        p90 = [round(s13.pct([dd[h]["sim_s"] for h in both], 0.9), 1),
               round(s13.pct([bb[h]["sim_s"] for h in both], 0.9), 1)]
        ms = statistics.median(sav) if sav else 0.0
        time[c] = {"median_sim_s_both": med, "p90_sim_s_both": p90, "median_saving_affected": round(ms, 4),
                   "affected_both_found": len(sav),
                   "gained": sorted(h for h in dd if dd[h]["outcome"] != "found" and bb[h]["outcome"] == "found"),
                   "lost": sorted(h for h in dd if dd[h]["outcome"] == "found" and bb[h]["outcome"] != "found"),
                   "met": med[1] <= med[0] and p90[1] <= p90[0] and ms >= 0.01}
    out["time"] = {**time, "met": all(v["met"] for v in time.values())}
    out["found"] = {"dev_branch": found, "met": all(v[1] >= v[0] for v in found.values())}
    out["wrong_found"] = {"rows": wrong, "met": not wrong}
    sv = {}
    for c in ("rooms", "rule"):
        sb = sc._rows(b / f"survey_{c}.json")
        sd = {sc._key(r): r for r in sc._rows(d / f"survey_{c}.json")}
        sv[c] = {"rows": len(sb), "identical": sum(sc._same(r, sd[sc._key(r)]) for r in sb)}
    out["surveys"] = {**sv, "met": all(v["rows"] == v["identical"] for v in sv.values())}
    for m in ("m3", "m2"):
        r = sc._c8(b / f"rows-{m}.jsonl", d / f"rows-{m}.jsonl")
        # This section's bar is found >= dev per arm only (3.192's C8 `met`
        # also asks that a changed row ended `exhausted` on dev).
        fb = r["found_branch_vs_dev"]
        out[m.upper()] = {"found_branch_vs_dev": fb, "changed": len(r["changed"]),
                          "met": all(v["branch"] >= v["dev"] for v in fb.values())}
    (HERE / "B-bars" / "summary.json").write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps(out, indent=1, default=str))


if __name__ == "__main__":
    main()
