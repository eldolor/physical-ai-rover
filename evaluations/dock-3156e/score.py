# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""
evaluations/dock-3156e/score.py -- 3.156.E's live bars scored from
`live.py`'s records (`live.jsonl`, the main run; `live-g4-300s.jsonl`, the
supplementary G4 run; `live-g5.jsonl`, G5's re-run). Pinned by
`tests/test_dock_live_3156e.py`. A record that never started, or lacks a
field a bar reads, counts against that bar rather than crashing the scorer.

    python evaluations/dock-3156e/score.py [live.jsonl [live-g4-300s.jsonl [live-g5.jsonl]]]
"""

import json
import math
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
EVERY_M, EVERY_DEG = 0.15, 15.0     # C2: every docked run
MEDIAN_M, MEDIAN_DEG = 0.10, 10.0   # C2: the median
# G5's "no motion" (amendment, 2026-11-20, before the re-run): within 3.84's
# live bar from the Stop's sending (coasting counts), and still from 1 s on.
STOP_MOVED_CM, STOP_TURNED_DEG = 5.0, 5.0
STILL_AFTER_S, STILL_CM, STILL_DEG = 1.0, 0.5, 0.5
WINDOW_S = 5.0


def rows(path):
    p = Path(path)
    return [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []


def _lapsed(r):
    ret = r.get("ret") or {}
    return "autonomy_lapsed" in f"{ret.get('reason') or ''} {r.get('end_reason') or ''}"


def _docked(r):
    return (r.get("ret") or {}).get("state") == "docked" and "err_m" in r and "err_deg" in r


def _median(xs):
    xs = list(xs)
    return round(statistics.median(xs), 3) if xs else None


def _still_after(r):
    """The largest motion from STILL_AFTER_S after the Stop to its 10 s."""
    track = [p for p in (r.get("after_stop_track") or []) if p[0] >= STILL_AFTER_S]
    if not track:
        return None
    x0, y0, h0 = track[0][1:]
    return (round(max(math.hypot(p[1] - x0, p[2] - y0) for p in track) * 100, 2),
            round(max(abs((p[3] - h0 + 180) % 360 - 180) for p in track), 2))


def _marker_lag(r):
    """1 when the one goal POST logged before the end marker is the return's
    own: the return was decided before the harness's poll saw it (the marker
    follows the poll), no goal POST came after the marker, and the only goal
    id polled is the return's, seen with the return set. Found after G5's
    re-run (drill 3: decided 0.017 s in, the marker at 0.29 s); disclosed in
    the plan file."""
    ret = r.get("ret") or {}
    seen = r.get("goals_seen") or {}
    decided, marked = ret.get("decided_at_s"), r.get("end_decided_s")
    return int(r.get("goal_posts_before_end") == 1 and r.get("goal_posts_after_end") == 0
               and isinstance(decided, (int, float)) and isinstance(marked, (int, float))
               and decided < marked and list(seen) == [ret.get("goal_id")]
               and all(v["return"] for v in seen.values()))


def _errors(found):
    found = [r for r in found if "start" not in r]
    ok = [r for r in found if _docked(r)]
    return {"n": len(found),
            "docked": len(ok),
            "in_every_bar": sum(bool(r.get("docked_in_bars")) for r in found),
            "median_m": _median(r["err_m"] for r in ok),
            "median_deg": None if not ok else round(statistics.median(r["err_deg"] for r in ok), 1),
            "worst_m": max((r["err_m"] for r in ok), default=None),
            "worst_deg": max((r["err_deg"] for r in ok), default=None),
            "contacts": sum(r.get("contacts") or 0 for r in found),
            "least_gap_cm": min((r["min_gap_cm"] for r in found if r.get("min_gap_cm") is not None),
                                default=None),
            "load_max": max((r.get("load_max") or 0 for r in found), default=None)}


def score(main_rows, supp_rows=(), g5_rows=()):
    finds = [r for r in main_rows if r["kind"] == "find"]
    first_drills = [r for r in main_rows if r["kind"] == "drill"]
    supp = [r for r in supp_rows if r["kind"] == "find"]
    drills = [r for r in g5_rows if r["kind"] == "drill"]

    g1 = [r for r in finds if r["house"] == "scaled_house"]
    docks = [r.get("dock") or {} for r in g1]
    whole = [d for d in docks if all(isinstance(d.get(k), (int, float)) for k in ("x_m", "y_m", "theta"))]
    spread_m = max((math.hypot(a["x_m"] - b["x_m"], a["y_m"] - b["y_m"])
                    for a in whole for b in whole), default=None)
    spread_deg = max((abs((a["theta"] - b["theta"] + 180) % 360 - 180)
                      for a in whole for b in whole), default=None)
    G1 = {"missions": len(g1),
          "saved_map": sum(d.get("source") == "saved_map" for d in docks),
          "off_dock_m": [r.get("start_off_dock_m") for r in g1],
          "dock_spread_m": spread_m, "dock_spread_deg": spread_deg,
          **_errors(g1)}
    G1["met"] = (G1["missions"] == 5 and G1["saved_map"] == 5 and len(whole) == 5
                 and spread_m <= 0.05 and spread_deg <= 2.0 and G1["in_every_bar"] == 5
                 and all(m is not None and 1.0 <= m <= 4.0 for m in G1["off_dock_m"]))

    G3 = {"main": _errors(finds), "supplementary": _errors(supp)}
    both = finds + supp
    ok = [r for r in both if _docked(r)]
    G3["met"] = (len(finds) == 10 and G3["main"]["n"] == 10 and G3["main"]["docked"] >= 9
                 and bool(ok)
                 and all(r["err_m"] <= EVERY_M and r["err_deg"] <= EVERY_DEG for r in ok)
                 and statistics.median(r["err_m"] for r in ok) <= MEDIAN_M
                 and statistics.median(r["err_deg"] for r in ok) <= MEDIAN_DEG
                 and sum(r.get("contacts") or 0 for r in both) == 0)

    # Every non-exploring mission run, the drills' finds too (review, 2026-11-20).
    g4 = both + first_drills + drills
    before = [r.get("goal_posts_before_end") for r in g4]
    lag = [_marker_lag(r) for r in g4]
    # Disclosed (Thermos, 2026-11-20): a mission whose end came in its first
    # seconds had no window to send an early goal; `with_window` counts those
    # that ran WINDOW_S or more before their end.
    # The cross-check: goal ids seen polling before the return, the goal
    # present at the mission's start left out (in the first run's records,
    # the one first seen in its first 0.1 s).
    # Rows with `prior_goal` already leave the start's goal out; older rows by
    # its first-0.1 s sighting.
    early_ids = sum(sum(1 for v in (r.get("goals_seen") or {}).values()
                        if not v["return"] and ("prior_goal" in r or v["t"] > 0.1)) for r in g4)
    # The bar as written (E's table): G3's ten. The wider set is disclosed.
    ten = [r.get("goal_posts_before_end") for r in finds]
    G4 = {"g3_ten": len(ten), "g3_ten_goal_posts_before_end": sum(b or 0 for b in ten),
          "g3_ten_unmeasured": sum(b is None for b in ten),
          "missions": len(g4), "goal_posts_before_end": sum(b or 0 for b in before),
          "the_returns_own_before_the_marker": sum(lag),
          "early": sum((b or 0) - x for b, x in zip(before, lag)),
          "with_window": sum((r.get("end_decided_s") or 0) >= WINDOW_S for r in g4),
          "pre_end_goal_ids_polled": early_ids,
          "unmeasured": sum(b is None for b in before),
          "goal_posts_after_end": [r.get("goal_posts_after_end") for r in g4],
          "end_decided_s": [r.get("end_decided_s") for r in g4]}
    G4["met"] = (len(ten) == 10 and G4["g3_ten_goal_posts_before_end"] == 0
                 and G4["g3_ten_unmeasured"] == 0)
    # Disclosed, not the verdict: the 26 with `_marker_lag`'s attribution
    # (written after the G5 re-run's row it explains).
    G4["wider_set_clean"] = G4["early"] == 0 and G4["unmeasured"] == 0 and early_ids == 0

    # The first run's drills are kept but not judged: every Stop landed
    # before the return had moved (ret.metres 0.0-0.01), so they could not fail.
    G5 = {"first_run_invalid": {"drills": len(first_drills),
                                "return_metres_at_stop": [(r.get("ret") or {}).get("metres")
                                                          for r in first_drills]},
          "drills": len(drills),
          "cancelled": sum((r.get("ret") or {}).get("state") == "cancelled" for r in drills),
          "stopped_while": [r.get("drill_phase") for r in drills],
          "return_moved_m_at_stop": [r.get("drill_return_moved_m") for r in drills],
          "return_turned_deg_at_stop": [r.get("drill_return_turned_deg") for r in drills],
          "moved_after_stop_cm": [r.get("moved_after_stop_cm") for r in drills],
          "turned_after_stop_deg": [r.get("turned_after_stop_deg") for r in drills],
          "still_after_1s": [_still_after(r) for r in drills],
          "autonomy_lapsed_in_g3": sum(_lapsed(r) for r in both)}
    # The amendment's gate and the Stop as the cause, scored, not only run.
    G5["gate_kept"] = sum(
        r.get("drill_phase") in ("driving", "creep", "turn")
        and ((r.get("drill_return_moved_m") or 0) >= 0.10 or (r.get("drill_return_turned_deg") or 0) >= 15.0)
        and str((r.get("ret") or {}).get("reason") or "").startswith("stop") and not r.get("stop_error")
        for r in drills)
    G5["met"] = (G5["drills"] == 5 and G5["cancelled"] == 5 and G5["gate_kept"] == 5
                 and all(m is not None and m <= STOP_MOVED_CM for m in G5["moved_after_stop_cm"])
                 and all(t is not None and t <= STOP_TURNED_DEG for t in G5["turned_after_stop_deg"])
                 and all(st is not None and st[0] <= STILL_CM and st[1] <= STILL_DEG
                         for st in G5["still_after_1s"])
                 and G5["autonomy_lapsed_in_g3"] == 0)
    return {"G1": G1, "G3": G3, "G4_live": G4, "G5": G5}


if __name__ == "__main__":
    a = sys.argv[1:]
    main_path = a[0] if a else HERE / "live.jsonl"
    supp_path = a[1] if len(a) > 1 else HERE / "live-g4-300s.jsonl"
    g5_path = a[2] if len(a) > 2 else HERE / "live-g5.jsonl"
    print(json.dumps(score(rows(main_path), rows(supp_path), rows(g5_path)), indent=1))
