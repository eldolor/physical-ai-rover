# SPDX-License-Identifier: AGPL-3.0-only
# Copyright (C) 2026 Anshu Gaind
"""3.141.E criterion 3 (amendment 5): the unclamped vets' cost from one
traced window's `trace3141.jsonl` (or `.gz`), every vet a row
(`TRACE_3141_VET_MS=0`). Unclamped = an empty reason (the command passed
unchanged). p50 / p90 / max on the loop (`MainThread`), in the threadpool
(any other thread) and over all; the bar is p90 <= 3 ms and max <= 15 ms on
each. The window is the steady window's own stamps (`window_start`,
`window_end` in the run's board directory) when given.

    python evaluations/wheel-loop-3141/vet_rows.py <board dir>
"""

import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "jetson-perf-3136"))
from scorers import percentile as pct  # noqa: E402  (one percentile rule: nearest rank)

P90_BAR_MS, MAX_BAR_MS = 3.0, 15.0


def stats(v):
    return {"n": len(v), "p50_ms": pct(v, 50), "p90_ms": pct(v, 90), "max_ms": max(v) if v else None,
            "met": bool(v) and pct(v, 90) <= P90_BAR_MS and max(v) <= MAX_BAR_MS}


def rows(board: Path):
    f = board / "trace3141.jsonl"
    opener = open
    if not f.exists():
        f, opener = board / "trace3141.jsonl.gz", gzip.open
    with opener(f, "rt") as fh:
        for line in fh:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def score(board: Path) -> dict:
    def stamp(name):
        p = board / name
        return float(p.read_text().split()[0]) if p.exists() else None
    t0, t1 = stamp("window_start"), stamp("window_end")
    loop, pool, clamped, parts, under_1ms = [], [], 0, {}, 0
    for r in rows(board):
        if r.get("k") != "vet" or (t0 is not None and r["t"] < t0) or (t1 is not None and r["t"] > t1):
            continue
        under_1ms += r["ms"] < 1.0
        if r.get("reason"):
            clamped += 1
            continue
        (loop if r.get("thread") == "MainThread" else pool).append(r["ms"])
        if r["ms"] > P90_BAR_MS:
            for name, (ms, _calls) in (r.get("parts") or {}).items():
                parts[name] = parts.get(name, 0.0) + ms
    out = {"window": [t0, t1], "loop": stats(loop), "threadpool": stats(pool),
           "all": stats(loop + pool), "clamped_rows": clamped,
           # A trace at the tracer's default threshold (vets over 5 ms only)
           # has no fast rows: it cannot give these numbers (amendment 5).
           "every_vet_recorded": under_1ms > 0,
           "parts_over_bar_ms": dict(sorted(parts.items(), key=lambda kv: -kv[1])[:12])}
    out["met"] = out["every_vet_recorded"] and out["loop"]["met"] and out["threadpool"]["met"]
    return out


if __name__ == "__main__":
    print(json.dumps(score(Path(sys.argv[1])), indent=1))
