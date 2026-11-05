# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.F criterion 1 (F0): one traced steady window's late ticks against
the `/world/map` replies in flight. From the run directory's `summary.json`
(the robot server's wheel loop: late, moving, `max_dt_s`) and the board
directory's `trace3141.jsonl[.gz]` (the tracer's probe ticks and request
rows), inside the window's own stamps:

* the probe ticks over 80 and 100 ms (the tracer's own `dt_ms`), and how
  many overlap a `/world/map` reply (the tick's span [t - dt, t] meets the
  reply's [t0, t0 + ms]), against the chance level: the share of the window
  within a tick's length of a reply (each reply widened by dt);
* the map replies the tracer saw -- those over its REQ_MS (20 ms), the ones
  that can hold the loop: count, p50 / p90 / max, over 100 ms -- and all of
  explore's map fetches (the shim records each);
* the unclamped vets over 15 ms and how many overlap a map reply (4);
* whether P12's poller ran (`mappoll_on`) and the missions' outcomes.

    python evaluations/wheel-loop-3141/map_overlap.py <run dir>
"""

import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vet_rows import pct  # noqa: E402  (the harness's nearest-rank percentile)


def _rows(board: Path):
    f, opener = board / "trace3141.jsonl", open
    if not f.exists():
        f, opener = board / "trace3141.jsonl.gz", gzip.open
    if not f.exists():
        raise SystemExit(f"no stall trace in {board}: run the window with STEADY_TRACER=1")
    with opener(f, "rt") as fh:
        for line in fh:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def _stamp(board: Path, name: str):
    p = board / name
    return float(p.read_text().split()[0]) if p.exists() else None


def _union(spans, lo, hi) -> float:
    """Length of the spans' union, clipped to [lo, hi]."""
    total, cs, ce = 0.0, None, None
    for a, b in sorted((max(a, lo), min(b, hi)) for a, b in spans if b > lo and a < hi):
        if cs is None or a > ce:
            if cs is not None:
                total += ce - cs
            cs, ce = a, b
        else:
            ce = max(ce, b)
    return total + (ce - cs if cs is not None else 0.0)


def overlaps(span, spans) -> bool:
    a, b = span
    return any(s <= b and a <= e for s, e in spans)


def score(run: Path) -> dict:
    board = run / "board"
    t0, t1 = _stamp(board, "window_start"), _stamp(board, "window_end")
    if t0 is None or t1 is None:
        raise SystemExit(f"no window stamps in {board}")
    maps, ticks, slow_vets = [], [], []
    for r in _rows(board):
        k = r.get("k")
        if k == "req" and r.get("path") == "/world/map":
            a, b = r["t0"], r["t0"] + r["ms"] / 1000
            if b >= t0 and a <= t1:                 # any part inside the window
                maps.append((a, b))
        elif k == "tick" and t0 <= r["t"] <= t1:
            dt = float(r["dt_ms"])
            ticks.append(((r["t"] - dt / 1000, r["t"]), dt))
        elif k == "vet" and not r.get("reason") and r["ms"] > 15 and t0 <= r["t"] <= t1:
            slow_vets.append((r["t"] - r["ms"] / 1000, r["t"]))
    maps.sort()
    reply_ms = [(b - a) * 1000 for a, b in maps]
    summary = json.loads((run / "summary.json").read_text()) if (run / "summary.json").exists() else {}
    on = (board / "mappoll_on").read_text().strip() if (board / "mappoll_on").exists() else "1"
    out = {"mappoll_on": on == "1", "window": [t0, t1], "wheel": summary.get("wheel") or {},
           "missions": summary.get("missions") or {},
           "map_in_flight_share": round(_union(maps, t0, t1) / (t1 - t0), 4),
           "map_replies_over_req_ms": {"n": len(maps), "p50_ms": pct(reply_ms, 50),
                                       "p90_ms": pct(reply_ms, 90),
                                       "max_ms": max(reply_ms) if reply_ms else None,
                                       "over_100_ms": sum(m > 100 for m in reply_ms)},
           "explore_map_fetches": ((summary.get("rows") or {}).get("P12") or {}).get("explore_map_fetch"),
           "unclamped_vets_over_15_ms": {"n": len(slow_vets),
                                         "overlap_map": sum(overlaps(v, maps) for v in slow_vets)}}
    for bar in (80, 100):
        big = [span for span, dt in ticks if dt > bar]
        widened = [(a - bar / 1000, b) for a, b in maps]        # a tick of `bar` ms ending here meets one
        out[f"probe_ticks_over_{bar}_ms"] = {
            "n": len(big), "overlap_map": sum(overlaps(s, maps) for s in big),
            "chance": round(_union(widened, t0, t1) / (t1 - t0), 4)}
    return out


if __name__ == "__main__":
    print(json.dumps(score(Path(sys.argv[1])), indent=1))
