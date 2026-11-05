# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.F criterion 1 (F0): one traced steady window's late ticks against
the `/world/map` replies in flight. From the run directory's `summary.json`
(the robot server's wheel loop: late, moving, `max_dt_s`) and the board
directory's `trace3141.jsonl[.gz]` (the tracer's probe ticks and request
rows), inside the window's own stamps:

* the probe ticks over 80 and 100 ms, and how many overlap a `/world/map`
  reply (the tick's span [t - dt, t] meets the reply's [t0, t0 + ms]);
* the share of the window with a map reply in flight (the chance level);
* the map replies' count, p50 / p90 / max, and how many over 100 ms;
* the unclamped vets over 15 ms and how many overlap a map reply (4);
* whether P12's poller ran (`mappoll_on`).

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
    with opener(f, "rt") as fh:
        for line in fh:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def _stamp(board: Path, name: str):
    p = board / name
    return float(p.read_text().split()[0]) if p.exists() else None


def overlaps(span, spans) -> bool:
    a, b = span
    return any(s <= b and a <= e for s, e in spans)


def score(run: Path) -> dict:
    board = run / "board"
    t0, t1 = _stamp(board, "window_start"), _stamp(board, "window_end")
    rows = [r for r in _rows(board)]

    def inside(t):
        return t0 is not None and t1 is not None and t0 <= t <= t1

    maps = sorted((r["t0"], r["t0"] + r["ms"] / 1000) for r in rows
                  if r.get("k") == "req" and r.get("path") == "/world/map" and inside(r["t0"]))
    union, cs, ce = 0.0, None, None
    for a, b in maps:
        if cs is None or a > ce:
            if cs is not None:
                union += ce - cs
            cs, ce = a, b
        else:
            ce = max(ce, b)
    if cs is not None:
        union += ce - cs
    ticks = [(r["t"] - float(r["dt_ms"]) / 1000, r["t"]) for r in rows
             if r.get("k") == "tick" and inside(r["t"])]
    tick_ms = {span: (span[1] - span[0]) * 1000 for span in ticks}
    slow_vets = [(r["t"] - r["ms"] / 1000, r["t"]) for r in rows
                 if r.get("k") == "vet" and not r.get("reason") and r["ms"] > 15 and inside(r["t"])]
    reply_ms = [(b - a) * 1000 for a, b in maps]
    wheel = {}
    if (run / "summary.json").exists():
        wheel = json.loads((run / "summary.json").read_text()).get("wheel") or {}
    on = (board / "mappoll_on").read_text().strip() if (board / "mappoll_on").exists() else "1"
    out = {"mappoll_on": on == "1", "window": [t0, t1], "wheel": wheel,
           "map_in_flight_share": round(union / (t1 - t0), 4) if t0 and t1 else None,
           "map_replies": {"n": len(maps), "p50_ms": pct(reply_ms, 50), "p90_ms": pct(reply_ms, 90),
                           "max_ms": max(reply_ms) if reply_ms else None,
                           "over_100_ms": sum(m > 100 for m in reply_ms)},
           "unclamped_vets_over_15_ms": {"n": len(slow_vets),
                                         "overlap_map": sum(overlaps(v, maps) for v in slow_vets)}}
    for bar in (80, 100):
        big = [s for s in ticks if tick_ms[s] > bar]
        out[f"probe_ticks_over_{bar}_ms"] = {"n": len(big), "overlap_map": sum(overlaps(s, maps) for s in big)}
    return out


if __name__ == "__main__":
    print(json.dumps(score(Path(sys.argv[1])), indent=1))
