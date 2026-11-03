# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/jetson-fmea-3135/soak_score.py

3.135.C's scorer: criterion 1's bars (a)-(d), read off one soak's records
(`soak.py`'s, fetched by `run.py soak` into `<out>/board/`), exactly as
amendment 3 says they are read. Pure functions over rows, so the tests and
the planted check (`soak_plant.py`) feed them made-up and stand-in records.

    python evaluations/jetson-fmea-3135/soak_score.py <out>      # writes <out>/summary.json

* (a) MemAvailable minimum over the run (first mission's setup to the last
  mission's end), the lower of the 5-s account and the 1-s sysfs sampler;
  >= 500 MB.
* (b) the least-squares slope (MB/h) of each mission's MemAvailable minimum
  against the mission's midpoint, over missions starting after the first
  hour; >= -10 MB/h. And per role (memory_account's components, a role's
  processes summed, harness roles named apart), the median of the per-mission
  minimum RSS over hour 4's missions less that over hour 1's; <= 50 MB for
  every role present through whole missions in both hours.
* (c) 0 OOM kills (vmstat's counter and dmesg, the larger) and 0 late wheel
  ticks (the sum of every mission's robot server's `late_ticks`, read at the
  mission's end; a mission whose stack came up with no reading fails it).
* (d) >= 90% of missions `found` (a mission whose stack never came up counts,
  not found) and 0 contacts on ground truth, every mission's truth log whole
  (no hole over 0.2 s, no unjudged gap).
"""

from __future__ import annotations

import gzip
import json
import re
import statistics
import sys
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "evaluations" / "jetson-perf-3136"))

from tools.jetson.memory_account import component  # noqa: E402

FLOOR_MB = 500.0
SLOPE_MB_H = -10.0
GROWTH_MB = 50.0
FOUND_SHARE = 0.90
HOLE_S = 0.2
HOUR_S = 3600.0
SWEEP = {"found": 39, "n": 40, "missed": ["procthor:val:3 / box (exhausted)"],
         "how": "in-process explore sweep (FakeNav), the 40 house/target pairs, 2026-11-02"}
# The instrument and the harness: reported, never part of (b)'s per-role bar.
HARNESS_ROLES = ("measurement (sampler, tegrastats)", "harness", "cloud stand-in")
EXTRA_ROLES = [
    ("simulator body", r"truth_body\.py"),
    ("fake lidar", r"sim\.fake_lidar"),
    ("cloud stand-in", r"standin_cloud\.py"),
    ("harness", r"jetson-fmea-3135/soak\.(py|sh)|jetson-fmea-3135/soak_plant\.py"),
]
STANDIN = re.compile(r"soak_standin\.py --name (\S+)")


def role(p: dict) -> str:
    hay = f"{p.get('comm', '')} {p.get('cmd', '')}"
    m = STANDIN.search(hay)
    if m:
        return f"stand-in {m.group(1)}"
    c = component(p)
    if c != "system (other)":
        return c
    for name, pat in EXTRA_ROLES:
        if re.search(pat, hay):
            return name
    return f"other: {p.get('comm') or '?'}"


def opener(path: Path):
    path = Path(path)
    if path.exists():
        return open(path, "rt")
    gz = Path(str(path) + ".gz")
    if gz.exists():
        return gzip.open(gz, "rt")
    return None


def jsonl(path: Path) -> list:
    f = opener(path)
    if f is None:
        return []
    rows = []
    with f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except ValueError:
                continue            # a line cut by a kill
    return rows


def compact(rec: dict) -> dict:
    """One account sample as {t, phase, mem_mb, roles: {role: [rss_mb, gpu_mb, n]}}."""
    roles: dict = {}
    nv = rec.get("nvmap") or {}
    for p in rec.get("procs") or []:
        r = roles.setdefault(role(p), [0.0, 0.0, 0])
        r[0] += p.get("Rss", 0) / 1024
        r[1] += nv.get(str(p.get("pid")), 0) / 1024
        r[2] += 1
    return {"t": rec["t"], "phase": rec.get("phase", ""),
            "mem_mb": rec["meminfo"]["MemAvailable"] / 1024,
            "roles": {k: [round(v[0], 1), round(v[1], 1), v[2]] for k, v in roles.items()}}


def slope_per_h(points: list) -> Optional[float]:
    """Least-squares slope of [(t_s, y)] in y per hour; None under 2 points
    or with no spread in time."""
    if len(points) < 2:
        return None
    xs = [t / 3600.0 for t, _ in points]
    ys = [y for _, y in points]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx


def window(samples: list, t0: float, t1: float) -> list:
    return [s for s in samples if t0 <= s["t"] <= t1]


def mission_mins(samples: list, missions: list) -> list:
    """Per mission with a window: its MemAvailable minimum and, per role
    present in EVERY sample of the window, that role's minimum RSS."""
    out = []
    for mi in missions:
        if mi.get("t0") is None or mi.get("t1") is None:
            continue
        w = window(samples, mi["t0"], mi["t1"])
        if not w:
            out.append({"k": mi.get("k"), "t0": mi["t0"], "t1": mi["t1"], "n": 0,
                        "mem_min_mb": None, "rss_min_mb": {}})
            continue
        always = set.intersection(*(set(s["roles"]) for s in w))
        out.append({"k": mi.get("k"), "t0": mi["t0"], "t1": mi["t1"], "n": len(w),
                    "mem_min_mb": round(min(s["mem_mb"] for s in w), 1),
                    "rss_min_mb": {r: round(min(s["roles"][r][0] for s in w), 1) for r in always},
                    "rss_max_mb": {r: round(max(s["roles"][r][0] for s in w), 1) for r in always},
                    "gpu_min_mb": {r: round(min(s["roles"][r][1] for s in w), 1) for r in always}})
    return out


def bar_b(mins: list, run_start: float, hour_s: float = HOUR_S) -> dict:
    later = [(0.5 * (x["t0"] + x["t1"]) - run_start, x["mem_min_mb"]) for x in mins
             if x["t0"] >= run_start + hour_s and x["mem_min_mb"] is not None]
    # Scaled to a real hour: the planted check runs short "hours".
    s = slope_per_h([(t * 3600.0 / hour_s, y) for t, y in later])
    h1 = [x for x in mins if run_start <= x["t0"] < run_start + hour_s and x["n"]]
    h4 = [x for x in mins if run_start + 3 * hour_s <= x["t0"] < run_start + 4 * hour_s and x["n"]]
    growth, missing = {}, []
    for r in sorted({r for x in h1 + h4 for r in x["rss_min_mb"]}):
        a = [x["rss_min_mb"][r] for x in h1 if r in x["rss_min_mb"]]
        b = [x["rss_min_mb"][r] for x in h4 if r in x["rss_min_mb"]]
        if not a or not b:
            missing.append(r)
            continue
        growth[r] = {"hour1_median_mb": round(statistics.median(a), 1),
                     "hour4_median_mb": round(statistics.median(b), 1),
                     "growth_mb": round(statistics.median(b) - statistics.median(a), 1),
                     "n1": len(a), "n4": len(b)}
    judged = {r: g for r, g in growth.items() if r not in HARNESS_ROLES}
    over = sorted(r for r, g in judged.items() if g["growth_mb"] > GROWTH_MB)
    reasons = []
    if s is None:
        reasons.append("slope: under 2 missions after the first hour")
    elif s < SLOPE_MB_H:
        reasons.append(f"slope {s:.1f} MB/h < {SLOPE_MB_H}")
    if not h1 or not h4:
        reasons.append(f"missions in hour 1: {len(h1)}, hour 4: {len(h4)}")
    for r in over:
        reasons.append(f"{r} grew {judged[r]['growth_mb']} MB hour 1 -> 4")
    return {"slope_mb_per_h": None if s is None else round(s, 2), "slope_missions": len(later),
            "hour1_missions": len(h1), "hour4_missions": len(h4), "growth": growth,
            "roles_not_in_both_hours": missing, "over": over,
            "met": not reasons, "reasons": reasons}


def score(board: Path, *, hour_s: float = HOUR_S, sysfs: Optional[list] = None,
          desktop_load: Optional[list] = None) -> dict:
    board = Path(board)
    missions = [x for x in jsonl(board / "missions.jsonl")]
    rs = _f(board / "run_start")
    ended = _f(board / "run_end")          # written only when the loop ran its course
    re_ = ended if ended is not None else max((x.get("t1") or 0 for x in missions), default=None)
    have = rs is not None and re_ is not None
    compact_path = board / "memory.jsonl.gz"
    if compact_path.exists():
        samples = jsonl(board / "memory.jsonl")
    else:
        samples = [compact(r) for r in jsonl(board / "account.jsonl")]
    run = window(samples, rs, re_) if have else []
    sys_rows = [r for r in (sysfs or []) if have and rs <= r.get("t", 0) <= re_]
    # (a)
    acc_min = min((s["mem_mb"] for s in run), default=None)
    sys_min = min((r["mem_mb"] for r in sys_rows if isinstance(r.get("mem_mb"), (int, float))),
                  default=None)
    lows = [v for v in (acc_min, sys_min) if v is not None]
    a = {"account_min_mb": None if acc_min is None else round(acc_min, 1),
         "sysfs_min_mb": sys_min, "samples": len(run), "sysfs_samples": len(sys_rows)}
    a["min_mb"] = round(min(lows), 1) if lows else None
    a["met"] = a["min_mb"] is not None and a["min_mb"] >= FLOOR_MB and len(run) > 0
    # (b)
    mins = mission_mins(samples, missions)
    b = bar_b(mins, rs, hour_s) if rs is not None else {"met": False, "reasons": ["no run_start"]}
    # (c)
    ooms = [r["oom_kill"] for r in sys_rows if isinstance(r.get("oom_kill"), int)]
    vm = (ooms[-1] - ooms[0]) if ooms else None
    o0, o1 = _f(board / "oom_start"), _f(board / "oom_end")
    dm = max(0, int(o1 - o0)) if o0 is not None and o1 is not None else None
    late, unread = 0, []
    for x in missions:
        w = x.get("wheel_loop")
        if isinstance(w, dict) and isinstance(w.get("late_ticks"), int):
            late += w["late_ticks"]
        elif x.get("ready_t") is not None:
            unread.append(x.get("k"))
    c = {"oom_kills_vmstat": vm, "oom_kills_dmesg": dm, "late_ticks": late,
         "max_dt_s": max((x["wheel_loop"].get("max_dt_s") or 0 for x in missions
                          if isinstance(x.get("wheel_loop"), dict)), default=None),
         "moving_ticks": sum((x["wheel_loop"].get("moving_ticks") or 0) for x in missions
                             if isinstance(x.get("wheel_loop"), dict)),
         "wheel_unread_missions": unread,
         "late_by_mission": {x.get("k"): x["wheel_loop"]["late_ticks"] for x in missions
                             if isinstance(x.get("wheel_loop"), dict)
                             and x["wheel_loop"].get("late_ticks")}}
    kills = [v for v in (vm, dm) if v is not None]
    c["oom_kills"] = max(kills) if kills else None
    c["met"] = c["oom_kills"] == 0 and late == 0 and not unread
    # (d)
    n = len(missions)
    found = sum(x.get("outcome") == "found" for x in missions)
    contacts = sum(x.get("contacts") or 0 for x in missions)
    unjudged = [x.get("k") for x in missions if x.get("ready_t") is not None and (
        x.get("truth_hole_s") is None or x["truth_hole_s"] > HOLE_S
        or (x.get("truth_hole_all_s") or 0) > HOLE_S or x.get("gap_unknown"))]
    d = {"missions": n, "found": found, "share": round(found / n, 3) if n else None,
         "contacts": contacts, "truth_unjudged_missions": unjudged,
         "outcomes": _count(x.get("outcome") or ("error: " + str(x.get("error"))[:60])
                            for x in missions),
         "by_house": {h: f"{sum(x.get('outcome') == 'found' for x in missions if x['house'] == h)}"
                         f"/{sum(1 for x in missions if x['house'] == h)}"
                      for h in dict.fromkeys(x["house"] for x in missions)},
         "harness_stopped": [x.get("k") for x in missions if x.get("stopped_by_harness")],
         "errors": {x.get("k"): x["error"] for x in missions if x.get("error")},
         "sweep": SWEEP}
    d["met"] = bool(n) and found / n >= FOUND_SHARE and contacts == 0 and not unjudged
    span = (re_ - rs) if have else None
    # The loop wrote run_end, so it started missions for the whole 4 h.
    complete = ended is not None and span is not None and span >= 4 * hour_s
    return {"run_s": None if span is None else round(span, 1), "complete": complete,
            "missions_started_within_s": round(max((x["setup_t"] for x in missions), default=rs)
                                               - rs, 1) if rs is not None and missions else None,
            "a": a, "b": b, "c": c, "d": d,
            "met": complete and all(x["met"] for x in (a, b, c, d)),
            "durations_s": [round(x["t1"] - x["t0"], 1) for x in missions
                            if x.get("t0") and x.get("t1")],
            "per_mission": [{"k": x["k"], "house": x["house"], "target": x["target"],
                             "outcome": x.get("outcome"), "error": x.get("error"),
                             "s": round(x["t1"] - x["t0"], 1) if x.get("t0") and x.get("t1") else None,
                             "mem_min_mb": next((y["mem_min_mb"] for y in mins if y["k"] == x["k"]), None),
                             "late_ticks": (x.get("wheel_loop") or {}).get("late_ticks"),
                             "contacts": x.get("contacts"), "truth_hole_s": x.get("truth_hole_s")}
                            for x in missions],
            "desktop_load_max": max((v[1] for v in desktop_load or []), default=None)}


def write_compact(board: Path) -> Optional[Path]:
    """account.jsonl(.gz) -> memory.jsonl.gz (the committed form)."""
    rows = jsonl(Path(board) / "account.jsonl")
    if not rows:
        return None
    out = Path(board) / "memory.jsonl.gz"
    with gzip.open(out, "wt") as f:
        for r in rows:
            f.write(json.dumps(compact(r)) + "\n")
    return out


def _f(p: Path) -> Optional[float]:
    try:
        t = Path(p).read_text().strip()
        return float(t) if t else None
    except (OSError, ValueError):
        return None


def _count(items) -> dict:
    out: dict = {}
    for i in items:
        out[i] = out.get(i, 0) + 1
    return out


def main(argv=None) -> int:
    import boardrun as br
    argv = argv if argv is not None else sys.argv[1:]
    out = Path(argv[0])
    board = out / "board"
    rec = json.loads((out / "record.json").read_text()) if (out / "record.json").exists() else {}
    tele = board / "telemetry" / "sysfs.jsonl"
    write_compact(board)
    s = score(board, sysfs=br.read_sysfs(tele) if tele.exists() else [],
              desktop_load=rec.get("desktop_load"))
    s.update({"sha": rec.get("sha"), "cmd_exit": rec.get("cmd_exit"), "cmd_s": rec.get("cmd_s"),
              "power_mode": (board / "nvpmodel.txt").read_text().strip()
              if (board / "nvpmodel.txt").exists() else None,
              "ve_before": _json(board / "ve_before.json"), "ve_after": _json(board / "ve_after.json"),
              "cloud_calls": len(jsonl(board / "cloud_calls.jsonl")),
              "restore_actions": rec.get("restore_actions"), "restore_error": rec.get("restore_error"),
              "as_found_differences": rec.get("as_found_differences"),
              "left_as_found": rec.get("left_as_found")})
    if tele.exists():
        import validity
        s["telemetry"] = validity.telemetry_counts(br.read_sysfs(tele), _f(board / "run_start"),
                                                   _f(board / "run_end"))
    (out / "summary.json").write_text(json.dumps(s, indent=1, default=str))
    print(json.dumps({k: s[k] for k in ("met", "run_s")}, default=str))
    for k in "abcd":
        print(k, json.dumps({x: s[k][x] for x in s[k] if x not in ("growth", "per_mission")},
                            default=str)[:600])
    return 0


def _json(p: Path):
    try:
        return json.loads(Path(p).read_text())
    except (OSError, ValueError):
        return None


if __name__ == "__main__":
    raise SystemExit(main())
