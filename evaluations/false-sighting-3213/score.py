# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/false-sighting-3213/score.py -- 3.213.A's stop rule (A1-A3, as
the plan file defines them) and the false sightings' characterisation, from
measure.py's four files.

    python evaluations/false-sighting-3213/score.py DIR   (writes DIR/summary.json)
    python evaluations/false-sighting-3213/score.py probes DEV_DIR PROBE_DIR
        (3.213.B: each probe arm against step A's `dev`, the plan's selection
        rule; writes PROBE_DIR/summary.json)
"""

import json
import os
import statistics
import sys

GOOD_M, FALSE_M, AGREE_M = 0.5, 1.0, 0.5


def pct(xs, q):
    xs = sorted(xs)
    if not xs:
        return None
    k = (len(xs) - 1) * q
    lo, hi = int(k), min(int(k) + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def gbf(a):
    return (a["replaced"] and a["replaced_err_m"] is not None
            and a["replaced_err_m"] <= GOOD_M and a["err_m"] > FALSE_M)


def near(p, q, m):
    return ((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2) ** 0.5 <= m


def summarise(xs):
    xs = [x for x in xs if x is not None]
    if not xs:
        return {"n": 0}
    return {"n": len(xs), "median": round(statistics.median(xs), 3),
            "p10": round(pct(xs, 0.1), 3), "p90": round(pct(xs, 0.9), 3)}


def characterise(rows):
    false_s, good_s, agree_good, agree_false, events = [], [], 0, 0, 0
    for r in rows:
        S = r["sightings"]
        good_s += [s for s in S if s["err_m"] <= GOOD_M]
        prev_good = None
        for a in r["approaches"]:
            if gbf(a):
                events += 1
                # The sighting that made it: the same tick (approach `tick`).
                s = next((s for s in S if s["tick"] == a["tick"] and s["at"] == a["at"]), None)
                if s is not None:
                    false_s.append(s)
                later = [x for x in S if x["tick"] > a["tick"]]
                good = prev_good
                if good is not None and any(near(x["at"], good, AGREE_M) for x in later):
                    agree_good += 1
                if any(near(x["at"], a["at"], AGREE_M) for x in later):
                    agree_false += 1
            if a["err_m"] <= GOOD_M:
                prev_good = a["at"]

    def block(ss):
        hits = {}
        for s in ss:
            k = "target" if s["err_m"] <= GOOD_M else ("wall" if s["hit"] == "wall" else
                                                       "free" if s["hit"] == "free" else "object")
            hits[k] = hits.get(k, 0) + 1
        return {"n": len(ss), "range_m": summarise([s["range_m"] for s in ss]),
                "range_over_det": summarise([s["range_m"] / s["det_dist_m"] for s in ss
                                             if s.get("det_dist_m")]),
                "abs_bearing_deg": summarise([abs(s["bearing_deg"]) for s in ss
                                              if s["bearing_deg"] is not None]),
                "edge": sum(1 for s in ss if s["edge"]),
                "under_0_3_m": sum(1 for s in ss if s["range_m"] < 0.3),
                "lies_on": hits}

    return {"events": events, "false_replacing": block(false_s), "good": block(good_s),
            "later_agrees_with_replaced_good": agree_good,
            "later_agrees_with_false": agree_false}


def main(d):
    out = {}
    for c in ("rooms", "rule"):
        dev = {r["house"]: r for r in json.load(open(os.path.join(d, f"{c}_dev.json")))["rows"]}
        orc = {r["house"]: r for r in json.load(open(os.path.join(d, f"{c}_oracle.json")))["rows"]}
        both = [h for h in dev if dev[h]["outcome"] == "found" and orc[h]["outcome"] == "found"]
        affected = [h for h in dev if any(gbf(a) for a in dev[h]["approaches"])]
        aff_both = [h for h in affected if h in both]
        sav = [(dev[h]["sim_s"] - orc[h]["sim_s"]) / dev[h]["sim_s"] for h in aff_both]
        p90_dev = pct([dev[h]["sim_s"] for h in both], 0.9)
        p90_orc = pct([orc[h]["sim_s"] for h in both], 0.9)
        fd = sum(1 for r in dev.values() if r["outcome"] == "found")
        fo = sum(1 for r in orc.values() if r["outcome"] == "found")
        a1 = statistics.median(sav) if sav else 0.0
        a2 = (p90_dev - p90_orc) / p90_dev if p90_dev else 0.0
        gained = sorted(h for h in dev if dev[h]["outcome"] != "found" and orc[h]["outcome"] == "found")
        lost = sorted(h for h in dev if dev[h]["outcome"] == "found" and orc[h]["outcome"] != "found")
        aff_not_found = sorted(h for h in affected if dev[h]["outcome"] != "found")
        out[c] = {
            "rows": len(dev), "found_dev": fd, "found_oracle": fo, "gained": gained, "lost": lost,
            "affected_rows": len(affected), "affected_not_found_dev": aff_not_found,
            "affected_both_found": len(aff_both),
            "A1_median_saving": round(a1, 4),
            "saving_affected": summarise(sav),
            "saving_mean": round(statistics.mean(sav), 4) if sav else None,
            "seconds_saved_affected": summarise([dev[h]["sim_s"] - orc[h]["sim_s"] for h in aff_both]),
            "median_sim_s_both": [statistics.median([dev[h]["sim_s"] for h in both]),
                                  statistics.median([orc[h]["sim_s"] for h in both])],
            "p90_sim_s_both": [round(p90_dev, 1), round(p90_orc, 1)],
            "A2_p90_fall": round(a2, 4),
            "A3_found_gain": fo - fd,
            "total_sim_s_both": [round(sum(dev[h]["sim_s"] for h in both), 1),
                                 round(sum(orc[h]["sim_s"] for h in both), 1)],
            "oracle_kept": sum(r["oracle_kept"] for r in orc.values()),
            "unaffected_identical": sum(1 for h in dev if h not in affected
                                        and (dev[h]["outcome"], dev[h]["sim_s"]) == (orc[h]["outcome"], orc[h]["sim_s"])),
            "unaffected": len(dev) - len(affected),
            "triggers": {"A1": a1 >= 0.02, "A2": a2 >= 0.02, "A3": fo - fd >= 3},
            "characterise": characterise(list(dev.values())),
        }
    out["continue"] = any(any(out[c]["triggers"].values()) for c in ("rooms", "rule"))
    with open(os.path.join(d, "summary.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print(json.dumps(out, indent=1))


def probes(dev_dir, probe_dir):
    out = {}
    for arm in ("c1", "c2", "c1c2"):
        res, qual, saved = {}, True, 0.0
        for c in ("rooms", "rule"):
            dev = {r["house"]: r for r in json.load(open(os.path.join(dev_dir, f"{c}_dev.json")))["rows"]}
            pr = {r["house"]: r for r in json.load(open(os.path.join(probe_dir, f"{c}_{arm}.json")))["rows"]}
            both = [h for h in dev if dev[h]["outcome"] == "found" and pr[h]["outcome"] == "found"]
            affected = [h for h in dev if any(gbf(a) for a in dev[h]["approaches"])]
            sav = [(dev[h]["sim_s"] - pr[h]["sim_s"]) / dev[h]["sim_s"] for h in affected if h in both]
            fd = sum(1 for r in dev.values() if r["outcome"] == "found")
            fp = sum(1 for r in pr.values() if r["outcome"] == "found")
            med = [statistics.median([dev[h]["sim_s"] for h in both]),
                   statistics.median([pr[h]["sim_s"] for h in both])]
            p90 = [round(pct([dev[h]["sim_s"] for h in both], 0.9), 1),
                   round(pct([pr[h]["sim_s"] for h in both], 0.9), 1)]
            tot = [round(sum(dev[h]["sim_s"] for h in both), 1), round(sum(pr[h]["sim_s"] for h in both), 1)]
            ms = statistics.median(sav) if sav else 0.0
            ok = {"found": fp >= fd, "median": med[1] <= med[0], "p90": p90[1] <= p90[0],
                  "saving": ms >= 0.01}
            qual = qual and all(ok.values())
            saved += tot[0] - tot[1]
            res[c] = {"found": [fd, fp],
                      "gained": sorted(h for h in dev if dev[h]["outcome"] != "found" and pr[h]["outcome"] == "found"),
                      "lost": sorted(h for h in dev if dev[h]["outcome"] == "found" and pr[h]["outcome"] != "found"),
                      "median_sim_s_both": med, "p90_sim_s_both": p90, "total_sim_s_both": tot,
                      "median_saving_affected": round(ms, 4), "saving_affected": summarise(sav),
                      "changed_unaffected": sorted(h for h in dev if h not in affected and
                                                   (dev[h]["outcome"], dev[h]["sim_s"]) != (pr[h]["outcome"], pr[h]["sim_s"])),
                      "kept": sum(r["oracle_kept"] for r in pr.values()), "ok": ok}
        out[arm] = {**res, "qualifies": qual, "total_saved_s": round(saved, 1)}
    q = [a for a in out if out[a]["qualifies"]]
    best = max(q, key=lambda a: out[a]["total_saved_s"]) if q else None
    if best and best != "c1" and "c1" in q and out["c1"]["total_saved_s"] >= 0.99 * out[best]["total_saved_s"]:
        best = "c1"                                   # a tie within 1% goes to the one without C2
    out["winner"] = best
    with open(os.path.join(probe_dir, "summary.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    if sys.argv[1] == "probes":
        probes(sys.argv[2], sys.argv[3])
    else:
        main(sys.argv[1])
