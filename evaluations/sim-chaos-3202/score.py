# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/sim-chaos-3202/score.py

3.202's judge: reads a `chaos.py` run directory and scores criteria 1-6 (and
the planted runs of criterion 7) against the bars written in the section
file before any run. Pure functions over the records and the truth log
(`measure.py`'s), so the tests feed them made-up runs.

    python evaluations/sim-chaos-3202/score.py <out> [--json]

Writes <out>/score.json and prints one line per criterion.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / "evaluations" / "jetson-fmea-3135"))
sys.path.insert(0, str(ROOT))

import measure as m  # noqa: E402

LIDAR_STALE_S = 0.3          # robot/lidar_ld19.py STALE_S (fmea.py's)
FEEDBACK_STALE_S = 0.25      # robot/hardware_robot.py FEEDBACK_STALE_S
CELL_M = 0.30                # truth_body.py's scale: truth x/y are cells x CELL_M
WATCH_S = 6.0
HOLE_BAR_S = 0.2
FOUND_RADIUS_M = 1.0         # criterion 4(b)
FOUND_RATE_BAR = 0.90        # criterion 4(a)
MISSION_LIMIT_S = 1260.0     # criterion 4(c), amendment 1
RECOVERY_BAR_S = 90.0        # criterion 3
UNFIRED_BAR = 0.10           # criterion 1
MIN_FAULTS = 40              # criterion 1
LIFE_MIN_S = 1800.0          # criterion 5
SLOPE_BAR_MB_H = 20.0
GROWTH_BAR_MB = 50.0
NEVER_KILLED = ("body", "sensors", "sensors2")
NO_FOUND = ("K3", "K9", "K10", "K11")
FAULT_KINDS = ("K1", "K2", "K3", "K4", "K5", "K6", "K7", "K8", "K9", "K10", "K11")
EXCLUDE_FROM_RATE = ("K1", "K2", "K3", "K4", "K5", "K6", "K9", "K10", "K11")


def read_jsonl(path: Path) -> list:
    if not Path(path).exists():
        return []
    out = []
    for line in Path(path).read_text().splitlines():
        try:
            out.append(json.loads(line))
        except ValueError:
            continue
    return out


class Truth:
    """The truth rows, sorted, with windowed slices by bisect."""

    def __init__(self, rows: list):
        self.rows = sorted(rows, key=lambda r: r["t"])
        self.ts = [r["t"] for r in self.rows]

    def window(self, t0: float, t1: Optional[float] = None) -> list:
        i = bisect.bisect_left(self.ts, t0 - 1.0)            # a sample before t0 for forward_at
        j = len(self.ts) if t1 is None else bisect.bisect_right(self.ts, t1 + 1.0)
        return self.rows[i:j]


def target_m(house: str, target: str) -> Optional[tuple]:
    """The target's true centre in truth coordinates (metres)."""
    from sim.maps import build_world
    w = build_world(house)
    for (cx, cy), label in w.objects.items():
        if target in str(label).lower():
            return ((cx + 0.5) * CELL_M, (cy + 0.5) * CELL_M)
    return None


# ------------------------------------------------------------- per fault

def fault_measures(f: dict, tr: Truth, missions: dict) -> dict:
    """The measures one fault's bar reads, off the truth."""
    kind, t = f["kind"], f.get("fault_at")
    ms: dict = {}
    if t is None:
        return ms
    cleared = f.get("cleared_at", t)
    rows = tr.window(t, max(cleared, t) + WATCH_S + 11.0)
    mis = missions.get(f.get("mission")) or {}
    holes = []
    if kind in ("K1", "K2", "K3", "K4", "K5", "K6"):
        ms["forward_at_fault"] = m.forward_at(rows, t)
        ms["wheels_zero_s"] = m.wheels_zero_s(rows, t)
        ms["travel_cm"] = m.travel_cm(rows, t, t + WATCH_S)
        holes.append(m.gaps_s(rows, t - 1.0, t + WATCH_S))
    if kind == "K2":
        ms["drive_after_cont"] = m.nonzero_after(rows, f["cont_at"], f["cont_at"] + 10.0)
        holes.append(m.gaps_s(rows, f["cont_at"], f["cont_at"] + 10.0))
    if kind in ("K4", "K5"):
        stale = LIDAR_STALE_S if kind == "K4" else FEEDBACK_STALE_S
        ms["travel_stale_cm"] = m.travel_cm(rows, t + stale, f["cont_at"])
        holes.append(m.gaps_s(rows, t, f["cont_at"]))
    if kind in ("K7", "K8", "K9", "K10", "K11"):
        t1 = mis.get("t1", cleared)
        rows = tr.window(t, t1)
        ms["mission_s"] = round(t1 - t, 2)
        ms["contacts"] = m.contacts(rows, t, t1)
        ms["travel_cm"] = m.travel_cm(rows, t, t1)
        holes.append(m.gaps_s(rows, t, t1))
        if kind == "K11" and f.get("step1_at") is not None:
            ms["travel_after_hang_cm"] = m.travel_cm(rows, f["step1_at"] + 1.1, t1)
            ms["wheels_zero_s"] = m.wheels_zero_s(rows, f["step1_at"])     # criterion 6, reported
        ms["end_reason"] = mis.get("end_reason") or mis.get("error")
    ms["outcome"] = mis.get("outcome")
    ms["how"] = mis.get("how")
    ms["hung"] = bool(mis.get("hung"))
    ms["truth_hole_s"] = max(holes) if holes else None
    return ms


def judge_fault(f: dict, ms: dict, cfg: dict, proxy_rows: Optional[list] = None) -> dict:
    """The fault's own bar (the section's table). Every failed bar is a reason."""
    kind, r = f["kind"], []
    valid = True

    def bar(ok, why):
        if not ok:
            r.append(why)

    if f.get("errored"):
        return {"pass": False, "valid": False, "reasons": [f"the harness errored: {f['errored']}"]}
    if f.get("fault_at") is None:
        return {"pass": False, "valid": False, "reasons": ["never fired"]}
    if kind == "K11" and f.get("ended_before_step1"):
        valid = False
        r.append("invalid: the mission ended before its first step (the hang was never reached)")
    if kind in ("K1", "K2", "K3", "K4", "K5", "K6") and ms.get("forward_at_fault") is not True:
        valid = False
        r.append("invalid: the board was not driving forward when the fault fired")
    hole = ms.get("truth_hole_s")
    bar(hole is not None and hole <= HOLE_BAR_S, f"truth hole {hole} s in the fault's window")
    z = ms.get("wheels_zero_s")
    if kind in ("K1", "K2", "K6"):
        bar(z is not None and z <= 1.1, f"wheels zero after {z} s (bar 1.1)")
        bar(ms["travel_cm"] <= 15.0, f"moved {ms['travel_cm']} cm in 6 s after the fault (bar 15)")
    if kind == "K2":
        bar(ms["drive_after_cont"] == 0,
            f"{ms['drive_after_cont']} driving samples in the 10 s after SIGCONT (bar 0)")
    if kind == "K3":
        bar(z is not None and z <= 1.6, f"wheels zero after {z} s (bar 1.6)")
        bar(ms.get("outcome") != "found", "the killed robot's mission ended found")
    if kind in ("K4", "K5"):
        bar(ms["travel_stale_cm"] <= 2.0, f"moved {ms['travel_stale_cm']} cm while stale (bar 2)")
    if kind in ("K7", "K8"):
        bar(not ms.get("hung") and ms.get("outcome") not in (None, "lost", "harness_timeout"),
            f"the mission did not end on its own ({ms.get('outcome')}, hung={ms.get('hung')})")
        # The row: "not `failed` by an unhandled error". A designed end -- B3.2's
        # consecutive-failure budget ("... N times in a row") or B3.3's dead-man
        # ("brain loop hung") -- is handled; anything else that ends failed is not.
        why = str(ms.get("end_reason") or "")
        handled = "times in a row" in why or "brain loop hung" in why
        bar(ms.get("outcome") != "failed" or handled,
            f"the mission ended failed, not by a designed guard: {why[:160]!r}")
        bar(ms.get("contacts") == 0, f"{ms.get('contacts')} contact samples")
    if kind == "K8":
        five = _five_xx(proxy_rows or [], f.get("fault_at"), f.get("cleared_at"))
        ms["five_xx"] = five
        bar(five == 0, f"{five} 5xx answers from the robot server under duplicates")
    if kind in ("K9", "K10"):
        bar(ms.get("outcome") == "failed", f"the vision mission ended {ms.get('outcome')}, not failed")
        bar(ms["travel_cm"] <= 5.0, f"moved {ms['travel_cm']} cm over the mission (bar 5)")
    if kind == "K10":
        lim = cfg["vision_timeout_s"] * cfg["max_vision_failures"] + 30.0
        bar(ms["mission_s"] <= lim, f"the mission took {ms['mission_s']} s (bar {lim})")
    if kind == "K11":
        lim = cfg["drill_tick_timeout_s"] + 30.0
        bar(ms.get("outcome") != "found", "the hung mission ended found")
        bar(ms["mission_s"] <= lim, f"the mission took {ms['mission_s']} s (bar {lim})")
        a = ms.get("travel_after_hang_cm")
        bar(a is not None and a <= 2.0, f"moved {a} cm after the hung tick + 1.1 s (bar 2)")
    if kind in NO_FOUND:
        bar(ms.get("outcome") != "found", f"{kind}'s mission ended found")
    return {"pass": not r, "valid": valid, "reasons": r}


def _five_xx(proxy_rows: list, t0, t1) -> int:
    if t0 is None:
        return 0
    n = 0
    for p in proxy_rows:
        t = p.get("t")
        if t is None or t < t0 or (t1 is not None and t > t1):
            continue
        for k in ("fate", "dup_copy"):
            v = p.get(k)
            if isinstance(v, int) and v >= 500:
                n += 1
    return n


# ------------------------------------------------------------ the run

def slope_per_h(points: list) -> Optional[float]:
    """Least-squares slope of (t seconds, MB) in MB/h; None with < 3 points."""
    if len(points) < 3:
        return None
    xs = [p[0] / 3600.0 for p in points]
    ys = [p[1] for p in points]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return None
    return round(sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den, 2)


def memory(rss: list, t_start: float, t_end: float) -> dict:
    """Criterion 5's numbers from rss.jsonl."""
    lives: dict = {}
    for row in rss:
        for role, (pid, kb) in (row.get("rss") or {}).items():
            if kb is None:
                continue
            lives.setdefault((role, pid), []).append((row["t"], kb / 1024.0, row.get("mission")))
    out_lives, fails = [], []
    for (role, pid), pts in sorted(lives.items(), key=lambda kv: kv[1][0][0]):
        span = pts[-1][0] - pts[0][0]
        per_mission: dict = {}
        for t, mb, mis in pts:
            if mis is None:
                continue
            p = per_mission.setdefault(mis, [t, mb])
            p[1] = min(p[1], mb)
        mins = sorted(per_mission.values())
        s = slope_per_h(mins) if span >= LIFE_MIN_S else None
        rec = {"role": role, "pid": pid, "span_s": round(span), "missions": len(mins),
               "rss_first_mb": round(pts[0][1], 1), "rss_last_mb": round(pts[-1][1], 1),
               "rss_max_mb": round(max(p[1] for p in pts), 1), "slope_mb_h": s}
        out_lives.append(rec)
        if s is not None and s > SLOPE_BAR_MB_H:
            fails.append(f"{role} pid {pid}: per-mission minimum RSS slope {s} MB/h (bar {SLOPE_BAR_MB_H})")
    growth = {}
    for role in NEVER_KILLED:
        pts = [p for (r, _), ps in lives.items() if r == role for p in ps]
        first = [mb for t, mb, _ in pts if t <= t_start + LIFE_MIN_S]
        last = [mb for t, mb, _ in pts if t >= t_end - LIFE_MIN_S]
        if first and last:
            g = round(statistics.median(last) - statistics.median(first), 1)
            growth[role] = g
            if g > GROWTH_BAR_MB:
                fails.append(f"{role} grew {g} MB first 30 min -> last 30 min (bar {GROWTH_BAR_MB})")
        else:
            fails.append(f"{role}: no RSS in the first or last 30 minutes")
    ooms = [r["oom_kill"] for r in rss if r.get("oom_kill") is not None]
    oom = (ooms[-1] - ooms[0]) if ooms else None
    if oom:
        fails.append(f"{oom} OOM kills")
    return {"lives": out_lives, "growth_mb": growth, "oom_kills": oom, "fails": fails}


def score(out: Path) -> dict:
    out = Path(out)
    run = json.loads((out / "run.json").read_text())
    end = json.loads((out / "end.json").read_text()) if (out / "end.json").exists() else {}
    faults = read_jsonl(out / "faults.jsonl")
    missions_l = read_jsonl(out / "missions.jsonl")
    missions = {mm.get("id"): mm for mm in missions_l}
    proxy_rows = read_jsonl(out / "proxy.jsonl")
    tr = Truth(m.read_truth(out / "truth.jsonl") if (out / "truth.jsonl").exists() else [])
    cfg = run["config"]
    res: dict = {"run": {k: run.get(k) for k in ("started", "seed", "house", "kinds", "plant", "git", "host")},
                 "end": end}

    # Criterion 1
    fired = [f for f in faults if not f.get("unfired")]
    unfired = [f for f in faults if f.get("unfired")]
    per = []
    for f in fired:
        ms = fault_measures(f, tr, missions)
        v = judge_fault(f, ms, cfg, proxy_rows)
        per.append({"n": f.get("n"), "kind": f["kind"], "mission": f.get("mission"), "measures": ms,
                    "verdict": v})
    c1 = []
    if len(fired) < MIN_FAULTS and not run.get("plant"):
        c1.append(f"{len(fired)} faults fired (bar {MIN_FAULTS})")
    share = len(unfired) / max(1, len(faults))
    if share > UNFIRED_BAR:
        c1.append(f"{len(unfired)} of {len(faults)} draws unfired ({share:.0%}, bar 10%)")
    c1 += [f"fault {p['n']} {p['kind']}: {'; '.join(p['verdict']['reasons'])}"
           for p in per if not p["verdict"]["pass"]]
    by_kind = {}
    for p in per:
        k = by_kind.setdefault(p["kind"], [0, 0])
        k[0] += 1
        k[1] += p["verdict"]["pass"]
    res["c1"] = {"met": not c1, "fired": len(fired), "unfired": len(unfired),
                 "by_kind": {k: f"{v[1]}/{v[0]}" for k, v in sorted(by_kind.items())},
                 "fails": c1, "faults": per}

    # Criterion 2
    sp = m.span(tr.rows)
    contacts = m.contacts(tr.rows)
    hole = m.gaps_s(tr.rows, sp[0], sp[1]) if sp else math.inf
    c2 = []
    if contacts:
        c2.append(f"{contacts} contact samples")
    if hole > HOLE_BAR_S:
        c2.append(f"truth hole {hole} s (bar {HOLE_BAR_S})")
    unjudged = sum(1 for r in tr.rows if r.get("gap_cm") is None)
    if unjudged:
        c2.append(f"{unjudged} truth samples with no gap (contact unjudged)")
    res["c2"] = {"met": not c2, "contacts": contacts, "truth_hole_s": hole, "rows": len(tr.rows),
                 "hours": round((sp[1] - sp[0]) / 3600, 2) if sp else 0, "fails": c2}

    # Criterion 3
    c3, rec_s = [], []
    for f in fired:
        if f.get("recovery_error") or f.get("recovery_s") is None:
            c3.append(f"fault {f.get('n')} {f['kind']}: no recovery ({f.get('recovery_error')})")
        else:
            rec_s.append(f["recovery_s"])
            if f["recovery_s"] > RECOVERY_BAR_S:
                c3.append(f"fault {f.get('n')} {f['kind']}: recovery {f['recovery_s']} s (bar {RECOVERY_BAR_S})")
    alive = end.get("alive") or {}
    dead = [r for r, a in alive.items() if not a]
    if "error" in end:
        c3.append(f"the run ended with an error: {end['error']}")
    if dead:
        c3.append(f"dead at the end: {dead}")
    if not all((end.get("health") or {}).get(k) for k in ("robot", "brain")):
        c3.append(f"/health at the end: {end.get('health')}")
    res["c3"] = {"met": not c3, "recovery_s_max": max(rec_s) if rec_s else None,
                 "recovery_s_median": statistics.median(rec_s) if rec_s else None, "fails": c3}

    # Criterion 4
    tgt = target_m(run["house"], run["target"])
    rate_set = [mm for mm in missions_l if not set(mm.get("tags") or []) & set(EXCLUDE_FROM_RATE)
                and mm.get("how") not in ("preempted", "stopped_at_end")]
    found = [mm for mm in rate_set if mm.get("outcome") == "found"]
    rate = len(found) / len(rate_set) if rate_set else None
    c4, wrong = [], []
    if rate is None or rate < FOUND_RATE_BAR:
        c4.append(f"(a) found {len(found)}/{len(rate_set)} = {rate} (bar {FOUND_RATE_BAR})")
    for mm in missions_l:
        if mm.get("outcome") != "found":
            continue
        tags = set(mm.get("tags") or [])
        if tags & set(NO_FOUND):
            wrong.append(f"mission {mm.get('id')} found under {sorted(tags & set(NO_FOUND))}")
        et = mm.get("end_truth") or {}
        d = (math.hypot(et["x"] - tgt[0], et["y"] - tgt[1])
             if tgt and et.get("x") is not None else None)
        mm["target_dist_m"] = None if d is None else round(d, 2)
        if d is None or d > FOUND_RADIUS_M:
            wrong.append(f"mission {mm.get('id')} found {d if d is None else round(d, 2)} m from the target "
                         f"(bar {FOUND_RADIUS_M})")
    if wrong:
        c4.append(f"(b) {len(wrong)} wrong Found: " + "; ".join(wrong[:5]))
    hung = [mm.get("id") for mm in missions_l if mm.get("hung")]
    if hung:
        c4.append(f"(c) {len(hung)} missions hung past {MISSION_LIMIT_S} s: {hung[:5]}")
    outcomes: dict = {}
    for mm in rate_set:
        outcomes[mm.get("outcome")] = outcomes.get(mm.get("outcome"), 0) + 1
    res["c4"] = {"met": not c4, "missions": len(missions_l), "rate_set": len(rate_set),
                 "found": len(found), "rate": None if rate is None else round(rate, 3),
                 "outcomes": outcomes, "wrong_found": wrong, "hung": hung,
                 "found_dist_m": sorted(mm["target_dist_m"] for mm in missions_l
                                        if mm.get("target_dist_m") is not None),
                 "mission_s_median": statistics.median([mm["t1"] - mm["t0"] for mm in rate_set
                                                        if "t1" in mm]) if rate_set else None,
                 "fails": c4}

    # Amendment 1: true path per mission-minute, clean against recovery missions (reported)
    def per_min(group):
        vals = [m.travel_cm(tr.window(mm["t0"], mm["t1"]), mm["t0"], mm["t1"]) / max(1e-6, (mm["t1"] - mm["t0"]) / 60)
                for mm in group if "t1" in mm and mm["t1"] - mm["t0"] >= 60]
        return {"n": len(vals), "median_cm_per_min": round(statistics.median(vals), 1) if vals else None}
    clean = [mm for mm in rate_set if "recovery" not in (mm.get("tags") or [])
             and not set(mm.get("tags") or []) & {"K7", "K8"}]
    recov = [mm for mm in rate_set if "recovery" in (mm.get("tags") or [])]
    res["c4"]["path_per_min"] = {"clean": per_min(clean), "recovery": per_min(recov)}

    # Criterion 5
    rss = read_jsonl(out / "rss.jsonl")
    mem = memory(rss, run.get("t_start", sp[0] if sp else 0), end.get("t_end", sp[1] if sp else 0))
    res["c5"] = {"met": not mem["fails"], **mem}

    # Criterion 6 (reported)
    lat = {}
    for p in per:
        if p["kind"] in ("K1", "K2", "K6", "K11") and p["measures"].get("wheels_zero_s") is not None:
            lat.setdefault(p["kind"], []).append(p["measures"]["wheels_zero_s"])
    late = [((f.get("health_after") or {}).get("wheel_loop") or {}).get("late_ticks") for f in fired]
    res["c6"] = {"stop_latency_s": {k: {"n": len(v), "median": statistics.median(v), "max": max(v)}
                                    for k, v in lat.items()},
                 "late_ticks_seen_max": max([x for x in late if x is not None], default=None)}
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    res = score(Path(a.out))
    (Path(a.out) / "score.json").write_text(json.dumps(res, indent=1, default=str))
    if a.json:
        print(json.dumps(res, default=str))
        return 0
    for c in ("c1", "c2", "c3", "c4", "c5"):
        print(f"{c}: {'met' if res[c]['met'] else 'FAILED'}  " + "; ".join(res[c]["fails"][:6]))
    print("c1 by kind:", res["c1"]["by_kind"])
    print("c4:", {k: res["c4"][k] for k in ("missions", "rate_set", "found", "rate", "outcomes")})
    print("c6:", res["c6"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
