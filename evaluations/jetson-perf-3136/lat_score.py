# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-perf-3136/lat_score.py

3.136.C's scorer: P7, P6 and P5 from one `lat.py` run directory (the board's
`<out>/board`, or a desktop run's own directory), amendment 7's readings.

    python evaluations/jetson-perf-3136/lat_score.py <run-dir> [--plant]

* P7: `verdict_s` per arrival, p90 (bar 2.0 s).
* P6: per Stop, wheels zero = the truth's first sample from which both
  setpoints stay zero 0.5 s (3.135's `measure.wheels_zero_s`), from the
  harness's stamp just before it sent `/stop`; the goal's end = the first
  25 ms poll of `GET /world/goal` that read it not active. p99 of each
  (bars 150 ms and 250 ms).
* P5: per drop, the first scan the robot server held after the drop whose
  nearest return ahead was under SCAN_SEEN_M, to the forward speed zero
  (`forward_zero_s`); p99 (bar 100 ms); drop to zero beside it; contacts
  (true gap < 1 cm) in the 3 s after the drop.
* Validity (the section's rule): late wheel ticks summed over the stacks,
  OOM kills, 15 W, nav2's goals by amendments 5-6 per stack with
  amendment 7's person rule, VE ready and perception alive at each stack's
  end.
* `--plant` (criterion 3): each row's base and planted arms side by side;
  the planted arm's median less the base's must be 100 ms within 10%.
"""

from __future__ import annotations

import argparse
import gzip
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "jetson-fmea-3135"))
sys.path.insert(0, str(HERE))

import measure as m  # noqa: E402
import scorers  # noqa: E402

SCAN_SEEN_M = 0.40
PERSON_END_S = 3.0         # amendment 7: a goal nav2 ends this soon after a person's act is theirs
CONTACT_WINDOW_S = 3.0
# Amendment 9 (the user, 2026-11-07): from the next run on, an `approach`
# goal nav2 aborts counts as amendment 5 counts a frontier goal's. Runs that
# started before this instant (runs 1-2) are read without it.
AMENDMENT_9_FROM = 1794117600.0          # 2026-11-08 00:00 CST
TERMINAL = {"SUCCEEDED": "succeeded", "CANCELED": "canceled", "ABORTED": "aborted",
            4: "succeeded", 5: "canceled", 6: "aborted"}


def rows(path: Path) -> list:
    """A jsonl file, plain or gzipped (lat.sh gzips the big ones)."""
    for p in (path, Path(str(path) + ".gz")):
        if p.exists():
            op = gzip.open if p.suffix == ".gz" else open
            out = []
            with op(p, "rt") as f:
                for line in f:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        continue
            return out
    return []


FWD_ZERO_M_S = 0.005


def forward_zero_s(rows_: list, t0: float, hold_s: float = m.HOLD_ZERO_S):
    """P5's end (amendment 7): `wheels_zero_s` on the forward component,
    the mean of the two setpoints, at or under FWD_ZERO_M_S -- a pivot in
    place (equal and opposite) is not forward motion."""
    conv = [dict(r, sp=[0.0, 0.0] if (r["sp"][0] + r["sp"][1]) / 2 <= FWD_ZERO_M_S else r["sp"])
            for r in rows_]
    return m.wheels_zero_s(conv, t0, hold_s)


def truth(d: Path) -> list:
    return sorted((r for r in rows(d / "truth.jsonl") if "t" in r and "sp" in r), key=lambda r: r["t"])


def pct(vals, q):
    """A missing reading (the wheels never held zero, the goal never read
    ended) is the worst, not dropped: it counts as infinitely late."""
    v = [float("inf") if x is None else x for x in vals]
    return scorers.percentile(v, q) if v else None


def ms(x):
    if x is None:
        return None
    return "inf" if x == float("inf") else round(1000 * x, 1)


def summary(vals_s: list, q: float) -> dict:
    v = [x for x in vals_s if x is not None]
    return {"n": len(vals_s), "n_measured": len(v), f"p{int(q)}_ms": ms(pct(vals_s, q)),
            "p50_ms": ms(pct(vals_s, 50)), "max_ms": ms(max(v)) if v else None}


def p7(run: Path) -> dict:
    out = {}
    for tag in ("base", "plant"):
        v = [r["verdict_s"] for r in rows(run / "p7.jsonl") if r.get("tag") == tag]
        if v:
            out[tag] = {**summary(v, 90), "values_s": v}
    return out


def p6(run: Path) -> dict:
    out = {}
    cache = {}
    for r in rows(run / "p6.jsonl"):
        d = run / f"s{r['stack']:03d}"
        if d not in cache:
            cache[d] = truth(d)
        r["wheels_zero_s"] = m.wheels_zero_s(cache[d], r["t_stop_m"])
        out.setdefault(r["tag"], {}).setdefault(r["kind"], []).append(r)
    res = {}
    for tag, kinds in out.items():
        for kind, rs in kinds.items():
            res.setdefault(tag, {})[kind] = {
                "wheels_zero": summary([x["wheels_zero_s"] for x in rs], 99),
                **({"goal_end": summary([x["goal_end_s"] for x in rs], 99)} if kind == "goal" else {}),
                "trials": [{k: x.get(k) for k in ("wheels_zero_s", "goal_end_s", "stop_reply_s",
                                                    "stop_status", "verb_reason")} for x in rs]}
    return res


def p5(run: Path) -> dict:
    res = {}
    cache = {}
    for r in rows(run / "p5.jsonl"):
        if r.get("moving") is False:      # not driving forward at the drop: retried, not a trial
            continue
        d = run / f"s{r['stack']:03d}"
        if d not in cache:
            cache[d] = (truth(d), sorted((x for x in rows(d / "lat_shim.jsonl") if x.get("k") == "scan"),
                                          key=lambda x: x["t"]))
        tr, scans = cache[d]
        t0 = r["drop"]["t"]
        seen = next((s for s in scans if s["t"] > t0 and s.get("fwd_m") is not None
                     and s["fwd_m"] < SCAN_SEEN_M), None)
        after = [x for x in tr if t0 <= x["t"] <= t0 + CONTACT_WINDOW_S and x.get("gap_cm") is not None]
        trial = {"drop_to_scan_s": None if seen is None else round(seen["t"] - t0, 4),
                 "scan_to_zero_s": None if seen is None else forward_zero_s(tr, seen["t"]),
                 "drop_to_zero_s": forward_zero_s(tr, t0),
                 "clear_before": (lambda b: None if b is None or b.get("fwd_m") is None
                                  else b["fwd_m"] >= SCAN_SEEN_M)(
                     next((s for s in reversed(scans) if s["t"] <= t0), None)),
                 "T_cm": r["drop"]["T_cm"], "min_gap_cm": min((x["gap_cm"] for x in after), default=None),
                 "contacts": m.contacts(tr, t0, t0 + CONTACT_WINDOW_S)}
        res.setdefault(r["tag"], []).append(trial)
    return {tag: {"scan_to_zero": summary([t["scan_to_zero_s"] for t in ts], 99),
                  "drop_to_zero": summary([t["drop_to_zero_s"] for t in ts], 99),
                  "not_clear_before": sum(t["clear_before"] is False for t in ts),
                  # The stop came before the lidar showed the object (the sim's
                  # depth grid sees a drop at once): scan to zero reads ~0 then.
                  "stopped_before_scan": sum(t["drop_to_zero_s"] is not None and t["drop_to_scan_s"] is not None
                                             and t["drop_to_zero_s"] < t["drop_to_scan_s"] for t in ts),
                  "drop_to_scan": summary([t["drop_to_scan_s"] for t in ts], 99),
                  "contacts": sum(t["contacts"] for t in ts),
                  "min_gap_cm": min((t["min_gap_cm"] for t in ts if t["min_gap_cm"] is not None), default=None),
                  "trials": ts} for tag, ts in res.items()}


def nav(d: Path, approach_as_frontier: bool = False) -> dict:
    """Amendments 5-6 for one stack, with amendment 7: a goal nav2 ended
    ABORTED or CANCELED within PERSON_END_S after a person's act (a Stop, a
    verb, a D-pad hold) is the person's, counted neither way."""
    pace = rows(d / "pace.jsonl")
    # (start, end) of each act; a drop lasts until its object is put back.
    acts = [(r["t"], r.get("until") or r["t"]) for r in rows(d / "person.jsonl")]
    ended = {}
    for r in sorted((r for r in pace if r.get("k") == "nav"), key=lambda r: r["t"]):
        for gid, st in r.get("goals") or []:
            if TERMINAL.get(st) in ("aborted", "canceled") and gid not in ended:
                ended[gid] = r["t"]
    person = {g for g, t in ended.items() if any(a0 <= t <= a1 + PERSON_END_S for a0, a1 in acts)}
    kept = [dict(r, goals=[g for g in r.get("goals") or [] if g[0] not in person])
            if r.get("k") == "nav" else r for r in pace]
    sends = rows(d / "explore_sends.jsonl")
    if approach_as_frontier:              # amendment 9
        sends = [dict(r, sent=[[g, "frontier" if k == "approach" else k] for g, k in r.get("sent") or []])
                 for r in sends]
    out = scorers.nav_goals_amended(kept, sends, cancel_rows=rows(d / "explore_cancels.jsonl"))
    out["person"] = len(person)
    return out


def validity(run: Path) -> dict:
    stacks = rows(run / "stacks.jsonl")
    late = sum(((s.get("wheel_loop") or {}).get("late_ticks") or 0) for s in stacks)
    no_wheel = [s["dir"] for s in stacks if not s.get("wheel_loop")]
    started = (run / "run_start").read_text().strip() if (run / "run_start").exists() else ""
    am9 = bool(started) and float(started) >= AMENDMENT_9_FROM
    navs = {s["dir"]: nav(run / s["dir"], approach_as_frontier=am9) for s in stacks}
    succeeded = sum(n["succeeded"] for n in navs.values())
    aborted_other = sum(n["aborted_other"] for n in navs.values())
    f = lambda p: (run / p).read_text().strip() if (run / p).exists() else None  # noqa: E731
    o0, o1 = f("oom_start"), f("oom_end")
    oom = (int(o1) - int(o0)) if (o0 or "").isdigit() and (o1 or "").isdigit() else None
    power = f("nvpmodel.txt")
    reasons = []
    if late:
        reasons.append(f"{late} late wheel ticks")
    if no_wheel:
        reasons.append(f"no wheel loop read for {no_wheel}")
    if aborted_other:
        reasons.append(f"{aborted_other} nav2 goals aborted (not frontier, not cancelled, not a person's)")
    if not succeeded:
        reasons.append("no nav2 goal succeeded")
    if oom:
        reasons.append(f"{oom} OOM kills")
    if power is not None and "15W" not in power.replace(" ", ""):
        reasons.append(f"power mode {power!r}")
    if any(s.get("ve_ready") is False for s in stacks):
        reasons.append("VE not ready at a stack's end")
    if any(s.get("perception_alive") is False for s in stacks):
        reasons.append("the perception loop died")
    return {"valid": not reasons, "reasons": reasons, "late_ticks": late,
            "max_dt_s": max((((s.get("wheel_loop") or {}).get("max_dt_s")) or 0 for s in stacks), default=None),
            "moving_ticks": sum(((s.get("wheel_loop") or {}).get("moving_ticks") or 0) for s in stacks),
            "oom_kills": oom, "power": power, "stacks": len(stacks), "amendment_9": am9,
            "nav": {"succeeded": succeeded, "aborted_other": aborted_other,
                    "per_stack": navs}}


def planted(res: dict) -> dict:
    """Criterion 3: planted median less base median, per row, against 100 ms +-10%."""
    out = {}

    def judge(name, base, plant):
        if not isinstance(base, (int, float)) or not isinstance(plant, (int, float)):
            out[name] = {"base_ms": base, "plant_ms": plant, "met": None}
            return
        d = plant - base
        out[name] = {"base_p50_ms": base, "plant_p50_ms": plant, "read_ms": round(d, 1),
                     "error_pct": round(d - 100.0, 1), "met": abs(d - 100.0) <= 10.0}
    g = lambda d, *ks: (d or {}).get(ks[0]) if len(ks) == 1 else g((d or {}).get(ks[0]), *ks[1:])  # noqa: E731
    judge("P7 verdict_s", g(res["P7"], "base", "p50_ms"), g(res["P7"], "plant", "p50_ms"))
    judge("P6 goal: wheels zero", g(res["P6"], "base", "goal", "wheels_zero", "p50_ms"),
          g(res["P6"], "plant", "goal", "wheels_zero", "p50_ms"))
    judge("P6 goal: goal end", g(res["P6"], "base", "goal", "goal_end", "p50_ms"),
          g(res["P6"], "plant", "goal", "goal_end", "p50_ms"))
    judge("P6 verb: wheels zero", g(res["P6"], "base", "verb", "wheels_zero", "p50_ms"),
          g(res["P6"], "plant", "verb", "wheels_zero", "p50_ms"))
    judge("P5 drop to zero", g(res["P5"], "base", "drop_to_zero", "p50_ms"),
          g(res["P5"], "plant", "drop_to_zero", "p50_ms"))
    judge("P5 scan to zero (information)", g(res["P5"], "base", "scan_to_zero", "p50_ms"),
          g(res["P5"], "plant", "scan_to_zero", "p50_ms"))
    out["P5 scan to zero (information)"]["judged"] = False
    return out


def score(run: Path, plant: bool = False) -> dict:
    res = {"P7": p7(run), "P6": p6(run), "P5": p5(run), "validity": validity(run)}
    if plant:
        res["planted"] = planted(res)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--plant", action="store_true")
    a = ap.parse_args(argv)
    res = score(Path(a.run), a.plant)
    (Path(a.run) / "lat_summary.json").write_text(json.dumps(res, indent=1))
    brief = {k: v for k, v in res.items()}
    print(json.dumps(brief, indent=1)[:20000])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
